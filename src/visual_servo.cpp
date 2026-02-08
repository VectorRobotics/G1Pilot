#include <g1_pilot/g1_pilot.h>

#include "rclcpp/rclcpp.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/Path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <algorithm>

using namespace ArmPilot;

class VisualServo : public rclcpp::Node
{
public:
    VisualServo() : Node("visual_servoing")
    {
        /* Creaating a node that can perform visual servoing */

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
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("servo/joint_states",10);

        /* Subscribers */
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,handle_new_goal_pose_);
        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/feedback",10,handle_new_feedback_);
        // left_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/feedback/dex3/left/odom",10,
        //     [this](nav_msgs::msg::Odometry::UniquePtr msg){this->handle_odom_(msg, &(this->left_ee_pose_))});
        // right_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/feedback/dex3/right/odom",10,
        //     [this](nav_msgs::msg::Odometry::UniquePtr msg){this->handle_odom_(msg, &(this->right_ee_pose_))});

        /* Timer loops */
        control_loop_ = this->create_wall_timer(100ms, controller_);

        /* Initializing variables */
        goal_ = Eigen::MatrixXd::Identity(4,4);
        joints_ = arm_handle_->controller->get_joint_names();

    }


private:

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::ShatedPtr left_odom_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::ShatedPtr right_odom_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_loop_;

    // Arm Handle
    std::unique_ptr<G1DualArm> arm_handle_;

    // Variables for state tracking
    std::vector<double> current_configuration_;
    std::vector<double> current_configuration_vel_;
    Eigen::MatrixXd left_ee_pose_;
    Eigen::MatrixXd right_ee_pose_;

    // Variables as parameters
    Eigen::VectorXd left_tiny_side_vel_ = (Eigen::VectorXd(6) << 0, 0.2, 0, 0, 0, 0).finished();
    Eigen::VectorXd right_tiny_side_vel_ = (Eigen::VectorXd(6) << 0, -0.2, 0, 0, 0, 0).finished();

    // Variables
    Eigen::MatrixXd goal_;
    std::vector<Eigen::MatrixXd> left_trajectory_;
    std::vector<Eigen::MatrixXd> right_trajectory_;
    std::vector<Eigen::MatrixXd> left_traj_stack_;
    std::vector<Eigen::MatrixXd> right_traj_stack_;
    double left_error_;
    double right_error_;
    JointState cmd_;
    std::vector<std::string> joints_;


    void controller_(){

        if (left_traj_stack_.empty() && right_traj_stack_.empty()){
            cmd_ = arm_handle_->controller->get_grav_ff(
                current_configuration_
            );
        }
        else if (~left_traj_stack_.empty() && right_traj_stack_.empty()){
            cmd_ = arm_handle_->controller->control_left_arms(
                current_configuration_, 
                current_configuration_vel_,
                left_traj_stack_.back()
            );
        } 
        else if (left_traj_stack_.empty() && ~right_traj_stack_.empty()){
            cmd_ = arm_handle_->controller->control_right_arms(
                current_configuration_, 
                current_configuration_vel_,
                right_traj_stack_.back()
            );
        } 
        else {
            cmd_ = arm_handle_->controller->control_left_arms(
                current_configuration_, 
                current_configuration_vel_,
                left_traj_stack_.back(),
                right_traj_stack_.back()
            );
        }

        if (~left_traj_stack_.empty()){
            left_error_ = arm_handle_->controller->get_current_left_ee_error();
            if (left_error_<0.02) // 2 cm 
                left_traj_stack_.pop_back();
        }

        if (~right_traj_stack_.empty()){
            right_error_ = arm_handle_->controller->get_current_right_ee_error();
            if (right_error_<0.02) // 2 cm 
                right_traj_stack_.pop_back();
        }

        left_ee_pose_ = arm_handle_->controller->get_current_left_ee_pose();
        right_ee_pose_ = arm_handle_->controller->get_current_right_ee_pose();

        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();

        message.name = cmd_.name;
        message.position = cmd_.position;
        message.velocity = cmd_.velocity;
        message.effort = cmd_.effort;

        joint_states_pub_->publish(message);

    }

    void handle_new_goal_pose_(geometry_msgs::msg::PoseStamped::UniquePtr msg){
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

        if (goal(1,3)<=0){
            right_trajectory_ = arm_handle_->motion_planner->planTrajectory(
                &goal_,
                &right_ee_pose_,
                &right_tiny_side_vel_
            )
            right_traj_stack_ = std::reverse(right_trajectory_.begin(), right_trajectory_.end());
        } else {
            left_trajectory_ = arm_handle_->motion_planner->planTrajectory(
                &goal_,
                &left_ee_pose_,
                &left_tiny_side_vel_
            )
            left_traj_stack_ = std::reverse(left_trajectory_.begin(), left_trajectory_.end());
        }
    }

    void handle_new_feedback_(sensor_msgs::msg::JointState::UniquePtr msg){
        std::vector<std::string>::iterator it;
        std::size_t index;
        for (int i = 0; i< joints_.size(); i++){
            it = std::find(msg->name.begin(), msg->name.end(), joint_[i])
            if (it!=msg->name.end()){
                index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
                current_configuration_[i] = msg->position[index];
                current_configuration_vel_[i] = msg->velocity[index];
            }
        }
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

int main(int arg, char* argv[])
{
    rclcpp::init(argc, argv);
    arclcpp::spin(std::make_shared<VisualServo>());
    rclcpp::shutdown();
    return 0;
}