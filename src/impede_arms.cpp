#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem> // Requires C++17
#include "sensor_msgs/msg/joint_state.hpp"

#include <g1_pilot/g1_pilot.h>


using namespace std::chrono_literals;
using namespace ArmPilot;

class ImpedeArms : public rclcpp::Node
{
public:
  ImpedeArms() : Node("ik_joint_state_publisher_cpp")
  {
    // Create a publisher on the "joint_states" topic
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("controller/joint_states", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "feedback", 10, 
        std::bind(&ImpedeArms::handle_new_state_, this, std::placeholders::_1)
    );
    
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

    t = 0;


  }

    std::unique_ptr<G1DualArm> arm_handle_;
    Eigen::VectorXd current_joint_angles_;
    Eigen::VectorXd current_joint_vels_;
    JointState result;
    sensor_msgs::msg::JointState reply;
    Eigen::Matrix4d left_target;
    Eigen::Matrix4d right_target;

    int t;

private:
  void handle_new_state_(sensor_msgs::msg::JointState::UniquePtr msg)
  {
    left_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, 0.2, 0.1);
    right_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, -0.2, 0.1);

    current_joint_angles_ = Eigen::Map<Eigen::VectorXd>(msg->position.data(),msg->position.size());
    current_joint_vels_ = Eigen::Map<Eigen::VectorXd>(msg->velocity.data(),msg->velocity.size());

    result = arm_handle_->controller->control_both_arms(
        current_joint_angles_, 
        current_joint_vels_,
        left_target, 
        right_target
    );

    reply = sensor_msgs::msg::JointState();

    reply.header.stamp = this->get_clock()->now();

    reply.name = result.name;
    reply.position = result.position;
    reply.velocity = result.velocity;
    reply.effort = result.effort;

    t+=1;
    RCLCPP_INFO(this->get_logger(), "Published IK Joint States at time: %d", t);

    publisher_->publish(reply);
  }

    Eigen::Matrix4d create_se3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
        transform.block<3, 1>(0, 3) = t;
        return transform;
    }

    // Helper function to create SE3 from quaternion components and translation
    Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz, 
                            double tx, double ty, double tz) {
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Vector3d t(tx, ty, tz);
        return create_se3(q, t);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImpedeArms>());
  rclcpp::shutdown();
  return 0;
}