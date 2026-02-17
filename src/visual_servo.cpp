#include <g1_pilot/g1_pilot.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "g1_pilot/action/move_arm.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

using namespace ArmPilot;
using namespace std::chrono_literals;
using MoveArm = g1_pilot::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

class VisualServo : public rclcpp::Node
{
public:
    VisualServo() : Node("visual_servoing")
    {
        /* Initialize arm handle */
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("g1_pilot");

        std::string default_asset_file = package_share_directory + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
        std::string default_asset_root = package_share_directory + "/assets/g1/";
        int default_n_dof = 29;

        this->declare_parameter<std::string>("asset_file", default_asset_file);
        this->declare_parameter<std::string>("asset_root", default_asset_root);
        this->declare_parameter<int>("num_dof", default_n_dof);

        RobotConfig config;
        config.asset_file = this->get_parameter("asset_file").as_string();
        config.asset_root = this->get_parameter("asset_root").as_string();
        config.NUM_DOF = this->get_parameter("num_dof").as_int();

        arm_handle_ = std::make_unique<G1DualArm>(&config);

        /* Publishers */
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("servo/joint_states", 10);

        /* Subscribers */
        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/feedback", 10,
            std::bind(&VisualServo::handle_new_feedback_, this, std::placeholders::_1)
        );

        /* Action server */
        action_server_ = rclcpp_action::create_server<MoveArm>(
            this, "move_arm",
            std::bind(&VisualServo::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&VisualServo::handle_cancel_, this, std::placeholders::_1),
            std::bind(&VisualServo::handle_accepted_, this, std::placeholders::_1)
        );

        /* Timer loop */
        control_loop_ = this->create_wall_timer(100ms, std::bind(&VisualServo::controller_, this));

        /* Initializing variables */
        joint_index_map_ = arm_handle_->controller->get_joint_idx_map();

        RCLCPP_INFO(this->get_logger(), "Visual servo node started with action server on /move_arm");
    }

private:

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

    // Action server
    rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleMoveArm> active_goal_handle_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_loop_;

    // Arm Handle
    std::unique_ptr<G1DualArm> arm_handle_;

    // State tracking
    Eigen::VectorXd current_configuration_;
    Eigen::VectorXd current_configuration_vel_;
    Eigen::MatrixXd left_ee_pose_;
    Eigen::MatrixXd right_ee_pose_;

    // Motion planning
    Eigen::VectorXd left_tiny_side_vel_ = (Eigen::VectorXd(6) << 0, 0.2, 0, 0, 0, 0).finished();
    Eigen::VectorXd right_tiny_side_vel_ = (Eigen::VectorXd(6) << 0, -0.2, 0, 0, 0, 0).finished();

    std::vector<Eigen::MatrixXd> left_trajectory_;
    std::vector<Eigen::MatrixXd> right_trajectory_;
    double left_error_ = 0.0;
    double right_error_ = 0.0;
    JointState cmd_;
    std::map<std::string, int> joint_index_map_;

    // ── Action callbacks ──

    rclcpp_action::GoalResponse handle_goal_(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const MoveArm::Goal>)
    {
        if (current_configuration_.size() == 0 ||
            left_ee_pose_.size() == 0 || right_ee_pose_.size() == 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: not ready (waiting for feedback + FK)");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: another trajectory is executing");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_(
        const std::shared_ptr<GoalHandleMoveArm>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancelling trajectory");
        left_trajectory_.clear();
        right_trajectory_.clear();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
    {
        active_goal_handle_ = goal_handle;
        const auto & target = goal_handle->get_goal()->target_pose;

        // Build goal SE3 from PoseStamped
        Eigen::Quaterniond q(
            target.pose.orientation.w,
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z
        );
        Eigen::MatrixXd goal = Eigen::MatrixXd::Identity(4, 4);
        goal.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        goal.block<3,1>(0,3) = Eigen::Vector3d(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z
        );

        RCLCPP_INFO(this->get_logger(), "Goal pose:\n%s",
            (std::ostringstream() << goal).str().c_str());

        // Freeze current EE pose and plan trajectory
        if (goal(1,3) <= 0) {
            RCLCPP_INFO(this->get_logger(), "Right EE pose:\n%s",
                (std::ostringstream() << right_ee_pose_).str().c_str());
            Eigen::MatrixXd start = right_ee_pose_;
            right_trajectory_ = arm_handle_->motion_planner->planTrajectory(
                &goal, &start, &right_tiny_side_vel_
            );
            std::reverse(right_trajectory_.begin(), right_trajectory_.end());
            RCLCPP_INFO(this->get_logger(), "Right arm: planned %zu waypoints", right_trajectory_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "Left EE pose:\n%s",
                (std::ostringstream() << left_ee_pose_).str().c_str());
            Eigen::MatrixXd start = left_ee_pose_;
            left_trajectory_ = arm_handle_->motion_planner->planTrajectory(
                &goal, &start, &left_tiny_side_vel_
            );
            std::reverse(left_trajectory_.begin(), left_trajectory_.end());
            RCLCPP_INFO(this->get_logger(), "Left arm: planned %zu waypoints", left_trajectory_.size());
        }
    }

    // ── Control loop ──

    void controller_()
    {
        if (current_configuration_.size() == 0) {
            return;
        }

        // Compute torques
        if (left_trajectory_.empty() && right_trajectory_.empty()) {
            cmd_ = arm_handle_->controller->get_grav_ff(current_configuration_);
        }
        else if (!left_trajectory_.empty() && right_trajectory_.empty()) {
            cmd_ = arm_handle_->controller->control_left_arm(
                current_configuration_, current_configuration_vel_,
                left_trajectory_.back()
            );
        }
        else if (left_trajectory_.empty() && !right_trajectory_.empty()) {
            cmd_ = arm_handle_->controller->control_right_arm(
                current_configuration_, current_configuration_vel_,
                right_trajectory_.back()
            );
        }
        else {
            cmd_ = arm_handle_->controller->control_both_arms(
                current_configuration_, current_configuration_vel_,
                left_trajectory_.back(), right_trajectory_.back()
            );
        }

        // Pop waypoints when reached
        if (!left_trajectory_.empty()) {
            left_error_ = arm_handle_->controller->get_current_left_ee_error();
            if (left_error_ < 0.02)
                left_trajectory_.pop_back();
        }
        if (!right_trajectory_.empty()) {
            right_error_ = arm_handle_->controller->get_current_right_ee_error();
            if (right_error_ < 0.02)
                right_trajectory_.pop_back();
        }

        // Update FK-derived EE poses
        left_ee_pose_ = arm_handle_->controller->get_current_left_ee_pose();
        right_ee_pose_ = arm_handle_->controller->get_current_right_ee_pose();

        // Publish joint commands
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();
        message.name = cmd_.name;
        message.position = cmd_.position;
        message.velocity = cmd_.velocity;
        message.effort = cmd_.effort;
        joint_states_pub_->publish(message);

        // Action feedback / result
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
            double error = std::max(left_error_, right_error_);
            uint32_t remaining = left_trajectory_.size() + right_trajectory_.size();

            if (active_goal_handle_->is_canceling()) {
                left_trajectory_.clear();
                right_trajectory_.clear();
                auto result = std::make_shared<MoveArm::Result>();
                result->success = false;
                result->message = "Cancelled";
                active_goal_handle_->canceled(result);
                active_goal_handle_.reset();
                return;
            }

            if (remaining == 0) {
                auto result = std::make_shared<MoveArm::Result>();
                result->success = true;
                result->message = "Trajectory complete";
                active_goal_handle_->succeed(result);
                active_goal_handle_.reset();
                RCLCPP_INFO(this->get_logger(), "Trajectory complete");
                return;
            }

            auto feedback = std::make_shared<MoveArm::Feedback>();
            feedback->error = error;
            feedback->waypoints_remaining = remaining;
            active_goal_handle_->publish_feedback(feedback);
        }
    }

    // ── Feedback subscriber ──

    void handle_new_feedback_(sensor_msgs::msg::JointState::UniquePtr msg)
    {
        current_configuration_ = Eigen::VectorXd(joint_index_map_.size());
        current_configuration_vel_ = Eigen::VectorXd(joint_index_map_.size());

        int idx = 0;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (joint_index_map_.count(msg->name[i]) > 0) {
                idx = joint_index_map_.at(msg->name[i]);
                current_configuration_[idx] = msg->position[i];
                current_configuration_vel_[idx] = msg->velocity[i];
            }
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServo>());
    rclcpp::shutdown();
    return 0;
}
