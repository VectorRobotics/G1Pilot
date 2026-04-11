#include <g1_pilot/g1_pilot.h>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "helper_funcs.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "humanoid_manipulation_interfaces/srv/plan_trajectory.hpp"

using namespace ArmPilot;

using PlanTrajectoryService = humanoid_manipulation_interfaces::srv::PlanTrajectory;

class PlanTrajectoryServer : public rclcpp::Node
{
public:
    PlanTrajectoryServer() : Node("plan_trajectory_server")
    {
        /* Initialize arm handle */
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

        /* TF2 */
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        /* Publishers */
        this->declare_parameter<std::string>("traj_topic", "traj");

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("traj_topic").as_string(), 10);

        /* Service server */
        service_ = this->create_service<PlanTrajectoryService>(
            "plan_trajectory",
            std::bind(&PlanTrajectoryServer::handle_service_, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Plan trajectory service server ready");
    }

private:
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Service server
    rclcpp::Service<PlanTrajectoryService>::SharedPtr service_;

    // Arm Handle
    std::unique_ptr<G1DualArm> arm_handle_;

    Eigen::MatrixXd get_pose_in_pelvis_(const geometry_msgs::msg::PoseStamped &pose_msg)
    {
        try {
            auto transformed = tf_buffer_->transform(pose_msg, "pelvis", tf2::TimePointZero, rclcpp::Duration(500ms));
            Eigen::Isometry3d pose_in_pelvis;
            tf2::fromMsg(transformed.pose, pose_in_pelvis);
            return pose_in_pelvis.matrix();
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return Eigen::MatrixXd();
        }
    }

    void handle_service_(
        const std::shared_ptr<PlanTrajectoryService::Request> request,
        std::shared_ptr<PlanTrajectoryService::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request for goal in frame: %s",
                     request->target_pose.header.frame_id.c_str());

        Eigen::MatrixXd goal_matrix = get_pose_in_pelvis_(request->target_pose);
        Eigen::MatrixXd start_matrix = get_pose_in_pelvis_(request->start_pose);

        if (goal_matrix.size() == 0 || start_matrix.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform poses to pelvis frame");
            response->success = false;
            return;
        }

        auto trajectory = arm_handle_->motion_planner->planTrajectory(
                &goal_matrix, &start_matrix);

        response->success = true;
        response->trajectory = convertToPath(trajectory, "pelvis", this->get_clock()->now());

        path_pub_->publish(response->trajectory);

        RCLCPP_INFO(this->get_logger(), "Planned trajectory with %zu waypoints",
                     response->trajectory.poses.size());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanTrajectoryServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
