#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem> // Requires C++17
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <g1_pilot/g1_pilot.h>

#include "helper_funcs.h"


using namespace std::chrono_literals;
using namespace ArmPilot;

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher() : Node("ik_joint_state_publisher_cpp")
  {
    // Create a publisher on the "joint_states" topic
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("position_control", 10);

    // Timer to call the callback at 10Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&JointStatePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "IK Joint State Publisher Node has started.");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("g1_pilot");
    
    std::string default_asset_file = package_share_directory + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
    std::string default_asset_root = package_share_directory + "/assets/g1/";

    this->declare_parameter<std::string>("asset_file", default_asset_file);
    this->declare_parameter<std::string>("asset_root", default_asset_root);

    RobotConfig config;
    config.asset_file = this->get_parameter("asset_file").as_string();
    config.asset_root = this->get_parameter("asset_root").as_string();
    config.NUM_DOF = 29;

    RCLCPP_INFO(this->get_logger(), "Initiaizing IK Classes");

    arm_handle_ = std::make_unique<G1DualArm>(&config);


    RCLCPP_INFO(this->get_logger(), "Initialized at time");

    left_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, 0.2, 0.1);
    right_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, -0.2, 0.1);
    goal_received_ = false;

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&JointStatePublisher::goal_pose_callback, this, std::placeholders::_1)
    );

  }

    // Create IK solver with collision detection
    std::unique_ptr<G1DualArm> arm_handle_;

    Eigen::Matrix4d left_target;
    Eigen::Matrix4d right_target;
    bool goal_received_;

private:
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    Eigen::Vector3d t(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );
    right_target = create_se3(q, t);
    goal_received_ = true;
  }

  void timer_callback()
  {
    if (!goal_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for goal pose on /goal_pose...");
      return;
    }

    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->get_clock()->now();

    auto result = arm_handle_->ik->solve_ik(
        left_target, right_target,
        nullptr, nullptr  // current q, dq
    );

    message.name = result.name;
    message.position = result.position;
    message.velocity = result.velocity;
    message.effort = result.effort;

    publisher_->publish(message);
  }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}