#ifndef HELPER_FUNCS_H_
#define HELPER_FUNCS_H_

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/clock.hpp>
#include <stdio.h>

Eigen::Matrix4d create_se3(const Eigen::Quaterniond &q, const Eigen::Vector3d &t)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;
    return transform;
}

Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz,
                            double tx, double ty, double tz)
{
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d t(tx, ty, tz);
    return create_se3(q, t);
}

nav_msgs::msg::Path convertToPath(const std::vector<Eigen::MatrixXd>& poses, std::string frame_id, builtin_interfaces::msg::Time stamp) {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped pose_stamped;

    path.header.frame_id = frame_id;
    path.header.stamp = stamp;
    
    pose_stamped.header = path.header;

    for (const auto& T : poses) {

        pose_stamped.pose.position.x = T(0, 3);
        pose_stamped.pose.position.y = T(1, 3);
        pose_stamped.pose.position.z = T(2, 3);

        Eigen::Quaterniond q(T.block<3, 3>(0, 0));
        q.normalize();
        
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }

    return path;
};

std::vector<Eigen::MatrixXd> convertToTrajectory(const nav_msgs::msg::Path& path) {
    std::vector<Eigen::MatrixXd> trajectory;
    trajectory.reserve(path.poses.size());

    for (const auto& pose_stamped : path.poses) {
        const auto& p = pose_stamped.pose;
        trajectory.push_back(create_se3(
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
            p.position.x, p.position.y, p.position.z));
    }

    return trajectory;
}

geometry_msgs::msg::PoseStamped convertToPoseStamped(
    const Eigen::MatrixXd& T,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = stamp;

    pose_stamped.pose.position.x = T(0, 3);
    pose_stamped.pose.position.y = T(1, 3);
    pose_stamped.pose.position.z = T(2, 3);

    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    q.normalize();

    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    return pose_stamped;
}

bool isWithinLimits(const Eigen::MatrixXd& T,
                    std::string& reason,
                    const Eigen::Vector3d& minPos = Eigen::Vector3d(0.1, -0.6, 0.0),
                    const Eigen::Vector3d& maxPos = Eigen::Vector3d(0.6, 0.6, 1.0),
                    double maxAngle = 1.04 // max rotation angle from identity (pi/2)
                )
{
    Eigen::Vector3d pos = T.block<3, 1>(0, 3);
    Eigen::AngleAxisd aa(T.block<3, 3>(0, 0).cast<double>());
    double angle = aa.angle();
    Eigen::Vector3d axis = aa.axis();

    reason = "pos=[" + std::to_string(pos[0]) + ", " + std::to_string(pos[1]) + ", " + std::to_string(pos[2]) + "]"
           + " axis-angle: angle=" + std::to_string(angle)
           + " axis=[" + std::to_string(axis[0]) + ", " + std::to_string(axis[1]) + ", " + std::to_string(axis[2]) + "]"
           + " | violated: ";

    std::string violations;

    const char* axisNames[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        if (pos[i] < minPos[i])
            violations += std::string(axisNames[i]) + " below min " + std::to_string(minPos[i]) + "; ";
        else if (pos[i] > maxPos[i])
            violations += std::string(axisNames[i]) + " above max " + std::to_string(maxPos[i]) + "; ";
    }

    if (angle > maxAngle)
        violations += "rotation angle " + std::to_string(angle) + " exceeds max " + std::to_string(maxAngle) + "; ";

    if (violations.empty()) {
        reason.clear();
        return true;
    }

    reason += violations;
    return false;
}

bool isWithinLimits(const Eigen::MatrixXd& T,
                    const Eigen::Vector3d& minPos = Eigen::Vector3d(0.1, -0.6, 0.0),
                    const Eigen::Vector3d& maxPos = Eigen::Vector3d(0.6, 0.6, 1.0),
                    double maxAngle = 1.57)
{
    std::string reason;
    return isWithinLimits(T, reason, minPos, maxPos, maxAngle);
}

#endif