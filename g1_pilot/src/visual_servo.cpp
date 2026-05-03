#include <g1_pilot/g1_pilot.h>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "helper_funcs.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tuple>

#include <algorithm>
#include <chrono>

using namespace HumanoidPilot;
using namespace std::chrono_literals;

class VisualServo : public rclcpp::Node
{
public:
    VisualServo() : Node("visual_servoing")
    {

        /* Initialize arm handle for arm functionalities */
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("g1_description");
        std::string g1_pilot_share_directory = ament_index_cpp::get_package_share_directory("g1_pilot");

        std::string default_asset_file = package_share_directory + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
        std::string default_asset_root = package_share_directory + "/assets/g1/";
        std::string default_config_file = g1_pilot_share_directory + "/config/g1.yaml";
        int default_n_dof = 29;

        this->declare_parameter<std::string>("asset_file", default_asset_file);
        this->declare_parameter<std::string>("asset_root", default_asset_root);
        this->declare_parameter<std::string>("config_file", default_config_file);
        this->declare_parameter<int>("num_dof", default_n_dof);

        RobotConfig config;
        config.asset_file = this->get_parameter("asset_file").as_string();
        config.asset_root = this->get_parameter("asset_root").as_string();
        config.config_file = this->get_parameter("config_file").as_string();
        config.NUM_DOF = this->get_parameter("num_dof").as_int();

        arm_handle_ = std::make_unique<Humanoid>(&config);

        /* TF2 */
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        /* Publishers */
        this->declare_parameter<std::string>("position_control_topic", "position_control");
        this->declare_parameter<std::string>("traj_topic", "traj");

        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(this->get_parameter("position_control_topic").as_string(),10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(this->get_parameter("traj_topic").as_string(),10);

        this->declare_parameter<double>("goal_cooldown", 1.0);

        /* Subscribers */
        this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
        this->declare_parameter<std::string>("feedback_topic", "feedback");

        left_goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_pose_topic").as_string() + "/left",10,
            [this](geometry_msgs::msg::PoseStamped::UniquePtr msg){ handle_new_goal_pose_(std::move(msg), true); }
        );
        right_goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_pose_topic").as_string() + "/right",10,
            [this](geometry_msgs::msg::PoseStamped::UniquePtr msg){ handle_new_goal_pose_(std::move(msg), false); }
        );

        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            this->get_parameter("feedback_topic").as_string(),10,
            std::bind(&VisualServo::handle_new_feedback_, this, std::placeholders::_1)
        );

        /* Timer loops */
        control_loop_ = this->create_wall_timer(50ms, std::bind(&VisualServo::controller_, this));


        /* Initializing variables */
        goal_ = Eigen::MatrixXd::Identity(4,4);

        last_left_goal_time_ = this->get_clock()->now();
        last_right_goal_time_ = this->get_clock()->now();
    }

    bool right_handed = true;


private:

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_loop_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Arm Handle
    std::unique_ptr<Humanoid> arm_handle_;

    // Variables for endeffector state
    Eigen::MatrixXd left_ee_pose_;
    Eigen::MatrixXd right_ee_pose_;
    Eigen::VectorXd left_ee_vel_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd right_ee_vel_ = Eigen::VectorXd::Zero(6);

    // // Variables as parameters
    // Eigen::VectorXd left_tiny_side_vel_ = (Eigen::VectorXd(6) << 0, 0.2, 0, 0, 0, 0).finished();
    // Eigen::VectorXd right_tiny_side_vel_ = (Eigen::VectorXd(6) << 0, -0.2, 0, 0, 0, 0).finished();
    
    // Variables
    Eigen::MatrixXd goal_;
    std::vector<Eigen::MatrixXd> left_trajectory_;
    std::vector<Eigen::MatrixXd> right_trajectory_;
    double left_error_;
    double right_error_;
    JointState cmd_;
    JointState current_state_;

    rclcpp::Time last_left_goal_time_;
    rclcpp::Time last_right_goal_time_;

    void controller_(){

        // Wait until first feedback message initializes the configuration
        if (current_state_.name.size() == 0){
            return;
        }

        if (left_trajectory_.empty() && right_trajectory_.empty()){
            cmd_ = arm_handle_->controller->control_no_arms(
                current_state_
            );
        }
        else if (!left_trajectory_.empty() && right_trajectory_.empty()){
            cmd_ = arm_handle_->controller->control_left_arm(
                current_state_, 
                left_trajectory_.back()
            );
        } 
        else if (left_trajectory_.empty() && !right_trajectory_.empty()){
            cmd_ = arm_handle_->controller->control_right_arm(
                current_state_, 
                right_trajectory_.back()
            );
        } 
        else {
            cmd_ = arm_handle_->controller->control_both_arms(
                current_state_, 
                left_trajectory_.back(),
                right_trajectory_.back()
            );
        }

        if (left_trajectory_.size()>1){
            left_error_ = arm_handle_->controller->get_current_left_ee_error();
            if (left_error_<0.02){ // 2 cm 
                left_trajectory_.pop_back();
            }
            RCLCPP_INFO(this->get_logger(), "Left error: %f", left_error_);
        }

        if (right_trajectory_.size()>1){
            right_error_ = arm_handle_->controller->get_current_right_ee_error();
            if (right_error_<0.02){ // 2 cm 
                right_trajectory_.pop_back();
            }
            RCLCPP_INFO(this->get_logger(), "Right error: %f", right_error_);
        }

        left_ee_pose_ = arm_handle_->controller->get_current_left_ee_pose();
        right_ee_pose_ = arm_handle_->controller->get_current_right_ee_pose();
        // left_ee_vel_ = arm_handle_->controller->get_current_left_ee_vel();
        // right_ee_vel_ = arm_handle_->controller->get_current_right_ee_vel();

        auto cmd_msg_ = sensor_msgs::msg::JointState();
        cmd_msg_.header.stamp = this->get_clock()->now();

        cmd_msg_.name = cmd_.name;
        cmd_msg_.position = cmd_.position;
        cmd_msg_.velocity = cmd_.velocity;
        cmd_msg_.effort = cmd_.effort;

        joint_states_pub_->publish(cmd_msg_);

    }

    void handle_new_goal_pose_(geometry_msgs::msg::PoseStamped::UniquePtr msg, bool left){
        // Wait until controller has computed EE poses from feedback
        if (left_ee_pose_.size() == 0 || right_ee_pose_.size() == 0){
            RCLCPP_WARN(this->get_logger(), "EE poses not yet initialized, ignoring goal");
            return;
        }

        auto& last_time = left ? last_left_goal_time_ : last_right_goal_time_;
        double cooldown = this->get_parameter("goal_cooldown").as_double();
        if (this->get_clock()->now() - last_time < rclcpp::Duration::from_seconds(cooldown)){
            RCLCPP_INFO(this->get_logger(), "Ignoring %s goal: less than %.1fs since last %s goal",
                left ? "left" : "right", cooldown, left ? "left" : "right");
            return;
        }

        last_time = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Received new goal in frame: %s", msg->header.frame_id.c_str());

        // Look up transform from the goal's parent frame to pelvis
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform(
                "pelvis", msg->header.frame_id, tf2::TimePointZero
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        // Convert TF to Eigen
        Eigen::Isometry3d tf_pelvis_from_source = tf2::transformToEigen(tf_stamped);

        // Convert incoming pose to Eigen
        Eigen::Quaterniond q_msg(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );
        Eigen::Isometry3d pose_in_source = Eigen::Isometry3d::Identity();
        pose_in_source.linear() = q_msg.normalized().toRotationMatrix();
        pose_in_source.translation() = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        // Transform goal into pelvis frame
        Eigen::Isometry3d goal_in_pelvis = tf_pelvis_from_source * pose_in_source;
        goal_.block<3,3>(0,0) = goal_in_pelvis.rotation();
        goal_.block<3,1>(0,3) = goal_in_pelvis.translation();

        std::string reason;
        if (!isWithinLimits(goal_, reason)){
            RCLCPP_WARN(this->get_logger(), "Goal out of bounds");
            return;
        }

        if (left){

            std::vector<Eigen::VectorXd> stash;
            std::tie(stash, left_trajectory_) = arm_handle_->motion_planner->planTrajectory(
                &goal_,
                &left_ee_pose_,
                &left_ee_vel_
            );
            path_pub_->publish(convertToPath(left_trajectory_, "pelvis", this->get_clock()->now()));
            std::reverse(left_trajectory_.begin(), left_trajectory_.end());

        } else {

            std::vector<Eigen::VectorXd> stash;
            std::tie(stash, right_trajectory_) = arm_handle_->motion_planner->planTrajectory(
                &goal_,
                &right_ee_pose_,
                &right_ee_vel_
            );
            path_pub_->publish(convertToPath(right_trajectory_, "pelvis", this->get_clock()->now()));
            std::reverse(right_trajectory_.begin(), right_trajectory_.end());

        }

        RCLCPP_INFO(this->get_logger(), "------/////|||||| New Trajectory Generated |||||||////////------");
        
    }

    void handle_new_feedback_(sensor_msgs::msg::JointState::UniquePtr msg){

        current_state_.name = msg->name;
        current_state_.position = msg->position;
        current_state_.velocity = msg->velocity;
        current_state_.effort = msg->effort;

    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServo>());
    rclcpp::shutdown();
    return 0;
}