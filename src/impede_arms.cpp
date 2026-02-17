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
	ImpedeArms() : Node("impedence_controller")
	{
		// Create a publisher on the "joint_states" topic
		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("controller/joint_states", 10);
		subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"feedback", 10,
			std::bind(&ImpedeArms::handle_new_state_, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Pilot Impedence Controller Node has started.");

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

		// Initialize joint names
		joint_index_map_ = arm_handle_->controller->get_joint_idx_map();

		// Initialize data storage variables
		current_joint_angles_ = Eigen::VectorXd(joint_index_map_.size());
		current_joint_vels_ = Eigen::VectorXd(joint_index_map_.size());

		t = 0;
	}

	Eigen::Matrix4d left_target;
	Eigen::Matrix4d right_target;

private:
	void handle_new_state_(sensor_msgs::msg::JointState::UniquePtr msg)
	{

		// Extract controller joints from full feedback
		int idx = 0;
		for (int i = 0; i < msg->name.size(); ++i)
		{
			if (joint_index_map_.count(msg->name[i]) > 0)
			{
				idx = joint_index_map_.at(msg->name[i]);
				current_joint_angles_(idx) = msg->position[i];
				current_joint_vels_(idx) = msg->velocity[i];
			}
		}

		left_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, 0.2, 0.1);
		right_target = create_se3(1.0, 0.0, 0.0, 0, 0.6, -0.4, 0.2);

		// Default pos:
		// left_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, 0.2, 0.1);
		// right_target = create_se3(1.0, 0.0, 0.0, 0, 0.2, -0.2, 0.1);

		result_ = arm_handle_->controller->control_both_arms(
			current_joint_angles_,
			current_joint_vels_,
			left_target,
			right_target);

		// result_ = arm_handle_->controller->get_grav_ff(current_joint_angles_);
		reply_ = sensor_msgs::msg::JointState();
		reply_.header.stamp = this->get_clock()->now();
		reply_.name = result_.name;
		reply_.position = result_.position;
		reply_.velocity = result_.velocity;
		reply_.effort = result_.effort;

		t += 1;

		publisher_->publish(reply_);
	}

	Eigen::Matrix4d create_se3(const Eigen::Quaterniond &q, const Eigen::Vector3d &t)
	{
		Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
		transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
		transform.block<3, 1>(0, 3) = t;
		return transform;
	}

	// Helper function to create SE3 from quaternion components and translation
	Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz,
							   double tx, double ty, double tz)
	{
		Eigen::Quaterniond q(qw, qx, qy, qz);
		Eigen::Vector3d t(tx, ty, tz);
		return create_se3(q, t);
	}
	
	// Private Variables
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;

	std::unique_ptr<G1DualArm> arm_handle_;

	std::vector<std::string> joint_names_;
	Eigen::VectorXd current_joint_angles_;
	Eigen::VectorXd current_joint_vels_;
	JointState result_;
	sensor_msgs::msg::JointState reply_;

	std::map<std::string, int> joint_index_map_;

	int t;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImpedeArms>());
	rclcpp::shutdown();
	return 0;
}