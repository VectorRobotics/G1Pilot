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

class JointTrajPublisher : public rclcpp::Node
{
public:
  JointTrajPublisher() : Node("joint_traj_publisher")
  {
    // Create a publisher on the "joint_states" topic
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("traj/joint_states", 10);

    // Timer to call the callback at 10Hz
    timer_ = this->create_wall_timer(3s, std::bind(&JointTrajPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Joint Traj Publisher Node has started.");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("g1_pilot");
    
    std::string default_asset_file = package_share_directory + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
    std::string default_asset_root = package_share_directory + "/assets/g1/";

    this->declare_parameter<std::string>("asset_file", default_asset_file);
    this->declare_parameter<std::string>("asset_root", default_asset_root);

    RobotConfig config;
    config.asset_file = this->get_parameter("asset_file").as_string();
    config.asset_root = this->get_parameter("asset_root").as_string();
    config.NUM_DOF = 29;

    RCLCPP_INFO(this->get_logger(), "Initiaizing Arm Pilot Classes");

    arm_ = std::make_unique<G1DualArm>(&config);

    RCLCPP_INFO(this->get_logger(), "Initialized at time");

    left_target = create_se3(0.7071, 0, 0.7071, 0, 0.5, 0.3, 1.2);
    right_target = create_se3(0.7071, 0, -0.7071, 0, 0.5, 0.3, 1.2);
    ext_force_left = Eigen::VectorXd::Zero(6);
    ext_force_right = Eigen::VectorXd::Zero(6);

    t = 0;


  }


    // Create IK solver with collision detection
    // G1_29_ArmIK_NoWrists* arm_;
    std::unique_ptr<G1DualArm> arm_;

    Eigen::Matrix4d left_target;
    Eigen::Matrix4d right_target;
    Eigen::VectorXd ext_force_left;
    Eigen::VectorXd ext_force_right;

    std::vector<Eigen::MatrixXd> traj;

    int t;

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();

    

    right_target.block<3,1>(0,3) = Eigen::Vector3d(0.01,0,0);

    traj = arm_->motion_planner->planTrajectory(
        new Eigen::MatrixXd(right_target)
    );

    for (auto pose: traj){
        auto result = arm_->ik->solve_ik(left_target, pose);
        auto q = Eigen::VectorXd::Map(result.position.data(), result.position.size());
        std::cout << "IK solution q: " << q.transpose() << std::endl;
        message.header.stamp = this->get_clock()->now();
        message.name = result.name;
        message.position = result.position;
        message.velocity = result.velocity;
        message.effort = result.effort;
        publisher_->publish(message);
    }
    t+=1;
    RCLCPP_INFO(this->get_logger(), "Published Trajectory Joint States at time: %d", t);
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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajPublisher>());
  rclcpp::shutdown();
  return 0;
}