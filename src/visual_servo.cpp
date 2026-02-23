#include <g1_pilot/g1_pilot.h>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "helper_funcs.h"

#include <algorithm>
#include <chrono>

using namespace ArmPilot;
using namespace std::chrono_literals;

class VisualServo : public rclcpp::Node
{
public:
    VisualServo() : Node("visual_servoing")
    {

        /* Initialize arm handle for arm functionalities */
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
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("effort_control",10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("traj",10);


        /* Subscribers */
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",10,
            std::bind(&VisualServo::handle_new_goal_pose_, this, std::placeholders::_1)
        );

        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/feedback",10,
            std::bind(&VisualServo::handle_new_feedback_, this, std::placeholders::_1)
        );

        // left_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/feedback/dex3/left/odom",10,
        //     [this](nav_msgs::msg::Odometry::UniquePtr msg){this->handle_odom_(msg, &(this->left_ee_pose_))});

        // right_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/feedback/dex3/right/odom",10,
        //     [this](nav_msgs::msg::Odometry::UniquePtr msg){this->handle_odom_(msg, &(this->right_ee_pose_))});


        /* Timer loops */
        control_loop_ = this->create_wall_timer(50ms, std::bind(&VisualServo::controller_, this));


        /* Initializing variables */
        goal_ = Eigen::MatrixXd::Identity(4,4);

		arm_handle_->controller->Kp_linear = 100;
        arm_handle_->controller->Kp_angular = 1;

        arm_handle_->controller->Kd_linear = 2;
        arm_handle_->controller->Kd_angular = 0.2;

        // arm_handle_->controller->Ki_linear = 10.0;
        // arm_handle_->controller->Ki_angular = 0.1;
    }


private:

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::ShatedPtr left_odom_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::ShatedPtr right_odom_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_loop_;

    // Arm Handle
    std::unique_ptr<G1DualArm> arm_handle_;

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

    std::stringstream left_ee_pose_debug;
    std::stringstream right_ee_pose_debug;


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
            if (left_error_<0.05){ // 2 cm 
                left_trajectory_.pop_back();
            }
        }

        if (right_trajectory_.size()>1){
            right_error_ = arm_handle_->controller->get_current_right_ee_error();
            if (right_error_<0.05){ // 2 cm 
                right_trajectory_.pop_back();
            }
        }

        left_ee_pose_ = arm_handle_->controller->get_current_left_ee_pose();
        right_ee_pose_ = arm_handle_->controller->get_current_right_ee_pose();
        // left_ee_vel_ = arm_handle_->controller->get_current_left_ee_vel();
        // right_ee_vel_ = arm_handle_->controller->get_current_right_ee_vel();

        auto reply = sensor_msgs::msg::JointState();
        reply.header.stamp = this->get_clock()->now();

        reply.name = cmd_.name;
        reply.position = cmd_.position;
        reply.velocity = cmd_.velocity;
        reply.effort = cmd_.effort;

        joint_states_pub_->publish(reply);

    }

    void handle_new_goal_pose_(geometry_msgs::msg::PoseStamped::UniquePtr msg){
        // Wait until controller has computed EE poses from feedback
        if (left_ee_pose_.size() == 0 || right_ee_pose_.size() == 0){
            RCLCPP_WARN(this->get_logger(), "EE poses not yet initialized, ignoring goal");
            return;
        }

        Eigen::Quaterniond q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );
        
        goal_.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        goal_.block<3,1>(0,3) = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        if (goal_(1,3)<=0){
            right_trajectory_ = arm_handle_->motion_planner->planTrajectory(
                &goal_,
                &right_ee_pose_,
                &right_ee_vel_
            );
            path_pub_->publish(convertToPath(right_trajectory_, "pelvis", this->get_clock()->now()));
            std::reverse(right_trajectory_.begin(), right_trajectory_.end());
        } else {
            left_trajectory_ = arm_handle_->motion_planner->planTrajectory(
                &goal_,
                &left_ee_pose_,
                &left_ee_vel_
            );
            path_pub_->publish(convertToPath(left_trajectory_, "pelvis", this->get_clock()->now()));
            std::reverse(left_trajectory_.begin(), left_trajectory_.end());
        }

        RCLCPP_INFO(this->get_logger(), "New goal processed");
        
    }

    void handle_new_feedback_(sensor_msgs::msg::JointState::UniquePtr msg){

        current_state_.name = msg->name;
        current_state_.position = msg->position;
        current_state_.velocity = msg->velocity;
        current_state_.effort = msg->effort;

    }

    // void handle_odom_(
    //     nav_msgs::msg::Odometry::UniquePtr msg,
    //     Eigen::MatrixXd* pose
    // ){
    //     *pose = Eigen::MatrixXd::Identity(4,4);

    //     Eigen::Quaterniond q(
    //         msg->pose.orientation.w,
    //         msg->pose.orientation.x,
    //         msg->pose.orientation.y,
    //         msg->pose.orientation.z
    //     );
        
    //     pose->block<3,3>(0,0) = q.normalized().toRotationMatrix();
    //     pose->block<3,1>(0,3) = Eigen::Vector3d(
    //         msg->pose.position.x,
    //         msg->pose.position.y,
    //         msg->pose.position.z
    //     );
    // }

    

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServo>());
    rclcpp::shutdown();
    return 0;
}