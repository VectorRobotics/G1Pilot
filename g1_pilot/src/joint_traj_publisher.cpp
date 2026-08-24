#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <g1_pilot/g1_pilot.h>
#include "helper_funcs.h"


using namespace std::chrono_literals;
using namespace HumanoidPilot;

class JointTrajPublisher : public rclcpp::Node
{
public:
    JointTrajPublisher() : Node("joint_traj_publisher")
    {
        std::string description_share = ament_index_cpp::get_package_share_directory("g1_description");
        std::string g1_pilot_share = ament_index_cpp::get_package_share_directory("g1_pilot");

        std::string default_asset_file = description_share + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
        std::string default_asset_root = description_share + "/assets/g1/";
        std::string default_config_file = g1_pilot_share + "/config/g1.yaml";

        this->declare_parameter<std::string>("asset_file", default_asset_file);
        this->declare_parameter<std::string>("asset_root", default_asset_root);
        this->declare_parameter<std::string>("config_file", default_config_file);
        this->declare_parameter<int>("num_dof", 29);
        this->declare_parameter<bool>("left_arm", true);
        this->declare_parameter<double>("waypoint_error_margin", 0.05);

        this->declare_parameter<std::string>("goal_pose_topic", "target_pose");
        this->declare_parameter<std::string>("position_control_topic", "position_control");
        this->declare_parameter<std::string>("feedback_topic", "feedback");
        this->declare_parameter<std::string>("traj_topic", "traj");

        RobotConfig config;
        config.asset_file = this->get_parameter("asset_file").as_string();
        config.asset_root = this->get_parameter("asset_root").as_string();
        config.config_file = this->get_parameter("config_file").as_string();
        config.NUM_DOF = this->get_parameter("num_dof").as_int();

        arm_ = std::make_unique<Humanoid>(&config);
        left_arm_ = this->get_parameter("left_arm").as_bool();
        // arm_->setLeftArm(left_arm_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        position_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            this->get_parameter("position_control_topic").as_string(), 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("traj_topic").as_string(), 10);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_pose_topic").as_string(), 5,
            std::bind(&JointTrajPublisher::handle_goal_pose_, this, std::placeholders::_1));

        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            this->get_parameter("feedback_topic").as_string(), 10,
            std::bind(&JointTrajPublisher::handle_feedback_, this, std::placeholders::_1));

        control_loop_ = this->create_wall_timer(
            50ms, std::bind(&JointTrajPublisher::controller_, this));

        RCLCPP_INFO(this->get_logger(),
            "Joint Traj Publisher ready (arm: %s)", left_arm_ ? "left" : "right");
    }

private:
    std::unique_ptr<Humanoid> arm_;
    bool left_arm_{true};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr position_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

    rclcpp::TimerBase::SharedPtr control_loop_;

    std::vector<std::string> trajectory_joint_names_;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;
    size_t next_waypoint_idx_ = 0;
    double error_ = 0.0;

    JointState current_state_;

    std::mutex feedback_mutex_;
    std::mutex traj_mutex_;

    Eigen::MatrixXd transform_to_pelvis_(const geometry_msgs::msg::PoseStamped &pose_msg)
    {
        try {
            auto stamped = pose_msg;
            stamped.header.stamp = rclcpp::Time(0);
            auto transformed = tf_buffer_->transform(stamped, "pelvis", tf2::durationFromSec(0.5));
            Eigen::Isometry3d pose_in_pelvis;
            tf2::fromMsg(transformed.pose, pose_in_pelvis);
            return pose_in_pelvis.matrix();
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return Eigen::MatrixXd();
        }
    }

    void handle_goal_pose_(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal pose in frame: %s", msg->header.frame_id.c_str());

        Eigen::MatrixXd start_matrix;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            if (current_state_.name.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                    "No joint feedback received yet — cannot plan");
                return;
            }
            arm_->update(current_state_);
            start_matrix = left_arm_
                ? arm_->get_current_left_ee_pose()
                : arm_->get_current_right_ee_pose();
        }

        Eigen::MatrixXd goal_matrix = transform_to_pelvis_(*msg);
        if (goal_matrix.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform goal pose to pelvis frame");
            return;
        }

        auto [joint_traj, cart_traj] = arm_->planTrajectory(
            &goal_matrix, &start_matrix);

        if (joint_traj.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "Trajectory planning failed: empty trajectory");
            return;
        }

        auto first = arm_->solve_ik(cart_traj.front(), left_arm_);

        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            trajectory_joint_names_ = first.name;
            trajectory_points_.clear();
            trajectory_points_.reserve(joint_traj.size());
            for (const auto &q : joint_traj) {
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions.assign(q.data(), q.data() + q.size());
                trajectory_points_.push_back(std::move(pt));
            }
            next_waypoint_idx_ = 0;
        }

        path_pub_->publish(convertToPath(cart_traj, "pelvis", this->get_clock()->now()));
        RCLCPP_INFO(this->get_logger(),
            "Planned trajectory with %zu waypoints", joint_traj.size());
    }

    void handle_feedback_(sensor_msgs::msg::JointState::UniquePtr msg)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        current_state_.name = msg->name;
        current_state_.position = msg->position;
        current_state_.velocity = msg->velocity;
        current_state_.effort = msg->effort;
    }

    void controller_()
    {
        std::scoped_lock lock(feedback_mutex_, traj_mutex_);

        if (current_state_.name.empty()) {
            return;
        }

        arm_->update(current_state_);

        if (trajectory_points_.empty() || next_waypoint_idx_ >= trajectory_points_.size()) {
            return;
        }

        auto current_target = trajectory_points_[next_waypoint_idx_];
        error_ = compute_position_error_(current_target.positions);

        if (error_ < this->get_parameter("waypoint_error_margin").as_double()) {
            ++next_waypoint_idx_;
            if (next_waypoint_idx_ >= trajectory_points_.size()) {
                RCLCPP_INFO(this->get_logger(), "Trajectory complete");
                return;
            }
            current_target = trajectory_points_[next_waypoint_idx_];
            error_ = compute_position_error_(current_target.positions);
        }

        RCLCPP_INFO(this->get_logger(), "Waypoint %zu/%zu, error: %f",
            next_waypoint_idx_ + 1, trajectory_points_.size(), error_);

        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = this->get_clock()->now();
        cmd.name = trajectory_joint_names_;
        cmd.position = current_target.positions;
        cmd.velocity = current_target.velocities;
        cmd.effort.assign(trajectory_joint_names_.size(), 0.0);
        position_pub_->publish(cmd);
    }

    double compute_position_error_(const std::vector<double> &positions)
    {
        double sum_sq = 0.0;
        size_t counted = 0;
        const size_t n = std::min(trajectory_joint_names_.size(), positions.size());
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < current_state_.name.size() && j < current_state_.position.size(); ++j) {
                if (trajectory_joint_names_[i] == current_state_.name[j]) {
                    const double d = positions[i] - current_state_.position[j];
                    sum_sq += d * d;
                    ++counted;
                    break;
                }
            }
        }
        return counted > 0 ? std::sqrt(sum_sq / counted) : 0.0;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajPublisher>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
