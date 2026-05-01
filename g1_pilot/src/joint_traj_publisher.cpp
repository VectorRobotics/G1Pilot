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

class JointTrajPublisher : public rclcpp::Node
{
public:
	JointTrajPublisher() : Node("joint_traj_publisher")
	{
		this->declare_parameter<std::string>("position_control_topic", "position_control");
		this->declare_parameter<std::string>("traj_topic", "traj");

		// Create a publisher on the "joint_states" topic
		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(this->get_parameter("position_control_topic").as_string(), 10);
		path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(this->get_parameter("traj_topic").as_string(), 10);

		// Timer to call the callback at 10Hz
		timer_ = this->create_wall_timer(3s, std::bind(&JointTrajPublisher::timer_callback, this));
		
		RCLCPP_INFO(this->get_logger(), "Joint Traj Publisher Node has started.");

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

		RCLCPP_INFO(this->get_logger(), "Initiaizing Arm Pilot Classes");

		arm_ = std::make_unique<Humanoid>(&config);

		RCLCPP_INFO(this->get_logger(), "Initialized at time");

		left_target = create_se3(1,0,0,0, 0.21, 0.149, 0.095);
		right_target = create_se3(1,0,0,0, 0.21, -0.149, 0.095);
		left_start = left_target;
		right_start = right_target;
		tiny_side_vel = Eigen::VectorXd::Zero(6);
		tiny_side_vel(1) =0.2; // Only for right arm

		t = 0;

	}

    std::unique_ptr<Humanoid> arm_;

    Eigen::Matrix4d left_target;
    Eigen::Matrix4d right_target;
    Eigen::Matrix4d left_start;
	Eigen::Matrix4d right_start;

    std::vector<Eigen::MatrixXd> traj;

    Eigen::VectorXd tiny_side_vel;

    int t;

private:
	void timer_callback()
	{
		auto message = sensor_msgs::msg::JointState();

		right_target.block<3,1>(0,3) = right_start.block<3,1>(0,3) + Eigen::Vector3d(0.09,-0.05,0.1);
		left_target.block<3,1>(0,3) = left_start.block<3,1>(0,3) + Eigen::Vector3d(0.09,0.05,0.1);

		// traj = arm_->motion_planner->planTrajectory(
		// 	new Eigen::MatrixXd(right_target),
		// 	new Eigen::MatrixXd(right_start),
		// 	&tiny_side_vel
		// );
		traj = arm_->motion_planner->planTrajectory(
			new Eigen::MatrixXd(left_target),
			new Eigen::MatrixXd(left_start),
			&tiny_side_vel
		);

		path_publisher_->publish(convertToPath(traj,"pelvis", this->get_clock()->now()));

		for (auto pose: traj){
			auto result = arm_->ik->solve_ik(pose, true);
			auto q = Eigen::VectorXd::Map(result.position.data(), result.position.size());
			// std::cout << "IK solution q: " << q.transpose() << std::endl;
			message.header.stamp = this->get_clock()->now();
			message.name = result.name;
			message.position = result.position;
			message.velocity = result.velocity;
			message.effort = result.effort;
			publisher_->publish(message);
			rclcpp::sleep_for(std::chrono::milliseconds(100));
		}
		t+=1;
		RCLCPP_INFO(this->get_logger(), "Published Trajectory Joint States at time: %d", t);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajPublisher>());
  rclcpp::shutdown();
  return 0;
}