#ifndef HELPER_FUNCS_H_
#define HELPER_FUNCS_H_

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <builtin_interfaces/msg/time.hpp>
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

bool isWithinLimits(const Eigen::MatrixXd& T, 
                    const Eigen::Vector3d& minPos = Eigen::Vector3d(0.1, -0.6, 0.0), 
                    const Eigen::Vector3d& maxPos = Eigen::Vector3d(0.6, 0.6, 1.0),
                    double maxRoll = 1.57, // pi/2
                    double maxPitch = 0.785, // pi/4
                    double maxYaw = 0.785  // pi/4
                ) 
{
    
    Eigen::Vector3d pos = T.block<3, 1>(0, 3);
    
    bool posValid = (pos.array() >= minPos.array()).all() && 
                    (pos.array() <= maxPos.array()).all();
    
    if (!posValid) return false;

    Eigen::Vector3d euler = T.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

    if (std::abs(euler[2]) > maxRoll)  return false; // Roll
    if (std::abs(euler[1]) > maxPitch) return false; // Pitch
    if (std::abs(euler[0]) > maxYaw)   return false; // Yaw

    return true;
}

#endif