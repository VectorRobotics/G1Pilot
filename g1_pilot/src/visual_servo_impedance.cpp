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

#include "helper_funcs.h"

using namespace std::chrono_literals;
using namespace HumanoidPilot;

class ImpedeArms : public rclcpp::Node

{
public:
	ImpedeArms() : Node("impedence_controller")
	{
		// Create a publisher on the "joint_states" topic
		this->declare_parameter<std::string>("feedback_topic", "feedback");
		this->declare_parameter<std::string>("effort_control_topic", "effort_control");

		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(this->get_parameter("effort_control_topic").as_string(), 10);
		subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			this->get_parameter("feedback_topic").as_string(), 10,
			std::bind(&ImpedeArms::handle_new_state_, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Pilot Impedence Controller Node has started.");

		// Intialize arm_handle
		std::string package_share_directory = ament_index_cpp::get_package_share_directory("g1_description");
		std::string g1_pilot_share_directory = ament_index_cpp::get_package_share_directory("g1_pilot");

		std::string default_asset_file = package_share_directory + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
		std::string default_asset_root = package_share_directory + "/assets/g1/";
		std::string default_config_file = g1_pilot_share_directory + "/config/g1.yaml";

		this->declare_parameter<std::string>("asset_file", default_asset_file);
		this->declare_parameter<std::string>("asset_root", default_asset_root);
		this->declare_parameter<std::string>("config_file", default_config_file);

		RobotConfig config;
		config.asset_file = this->get_parameter("asset_file").as_string();
		config.asset_root = this->get_parameter("asset_root").as_string();
		config.config_file = this->get_parameter("config_file").as_string();
		config.NUM_DOF = 29;

		RCLCPP_INFO(this->get_logger(), "Initiaizing Pilot Classes");

		arm_handle_ = std::make_unique<Humanoid>(&config);

		RCLCPP_INFO(this->get_logger(), "Initialized");

		t = 0;

		// with motion planner
        // arm_handle_->controller->Kp_linear = 120;
        // arm_handle_->controller->Kp_angular = 0.8;

        // arm_handle_->controller->Kd_linear = 2;
        // arm_handle_->controller->Kd_angular = 0.0;

		// without motion planner
		arm_handle_->controller->Kp_linear = 30;
        arm_handle_->controller->Kp_angular = 30;

        arm_handle_->controller->Kd_linear = 2;
        arm_handle_->controller->Kd_angular = 2;

	}

	Eigen::Matrix4d left_target;
	Eigen::Matrix4d right_target;

private:
	void handle_new_state_(sensor_msgs::msg::JointState::UniquePtr msg)
	{

		message_.name = msg->name;
		message_.position = msg->position;
		message_.velocity = msg->velocity;
		message_.effort = msg->effort;

		// left_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, 0.2, 0.1);
		// right_target = create_se3(1.0, 0.0, 0.0, 0, 0.6, -0.4, 0.2);

		// Default pos:
		left_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, 0.2, 0.1);
		right_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, -0.2, 0.1);

		result_ = arm_handle_->controller->control_both_arms(
			message_,
			left_target,
			right_target
		);

		reply_ = sensor_msgs::msg::JointState();
		reply_.header.stamp = this->get_clock()->now();
		reply_.name = result_.name;
		reply_.position = result_.position;
		reply_.velocity = result_.velocity;
		reply_.effort = result_.effort;

		t += 1;

		publisher_->publish(reply_);
	}
	
	// Private Variables
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;

	std::unique_ptr<Humanoid> arm_handle_;

	JointState message_;
	JointState result_;
	sensor_msgs::msg::JointState reply_;

	int t;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImpedeArms>());
	rclcpp::shutdown();
	return 0;
}