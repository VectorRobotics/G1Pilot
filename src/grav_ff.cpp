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

class GravFF : public rclcpp::Node

{
public:
	GravFF() : Node("gravity_feedforward")
	{
		// Create a publisher on the "joint_states" topic
		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("effort/joint_states", 10);
		subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"feedback", 10,
			std::bind(&GravFF::handle_new_state_, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Pilot Gravity Feedforward Node has started.");

		// Intialize arm_handle
		std::string package_share_directory = ament_index_cpp::get_package_share_directory("g1_pilot");

		std::string default_asset_file = package_share_directory + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
		std::string default_asset_root = package_share_directory + "/assets/g1/";

		this->declare_parameter<std::string>("asset_file", default_asset_file);
		this->declare_parameter<std::string>("asset_root", default_asset_root);

		RobotConfig config;
		config.asset_file = this->get_parameter("asset_file").as_string();
		config.asset_root = this->get_parameter("asset_root").as_string();
		config.NUM_DOF = 29;

		RCLCPP_INFO(this->get_logger(), "Initiaizing Pilot Classes");

		arm_handle_ = std::make_unique<G1DualArm>(&config);

		RCLCPP_INFO(this->get_logger(), "Initialized");

		t = 0;
	}

private:
	void handle_new_state_(sensor_msgs::msg::JointState::UniquePtr msg)
	{

		message_.name = msg->name;
		message_.position = msg->position;
		message_.velocity = msg->velocity;
		message_.effort = msg->effort;
		
		result_ = arm_handle_->grav_ff(message_);

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

	std::unique_ptr<G1DualArm> arm_handle_;

	JointState message_;
	JointState result_;
	sensor_msgs::msg::JointState reply_;

	int t;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GravFF>());
	rclcpp::shutdown();
	return 0;
}