#include "arm_control/ik_traj_tracker.h"

#include <pinocchio/algorithm/utils/motion.hpp>
#include <iostream>

namespace HumanoidPilot {
    
JointState IKTrajTracker::control_both_arms(
    JointState current_state,
    Eigen::Matrix4d desired_l_ee_pose,
    Eigen::Matrix4d desired_r_ee_pose,
    Eigen::VectorXd desired_l_ee_vel,
    Eigen::VectorXd desired_r_ee_vel
)
{
    update(current_state);
    
    desired_left_ee_pose_ = pinocchio::SE3(desired_l_ee_pose);
    desired_right_ee_pose_ = pinocchio::SE3(desired_r_ee_pose);

    double l_pos_err = 0.0, l_rot_err = 0.0, r_pos_err = 0.0, r_rot_err = 0.0;
    bool collision = false;
    result = ik_handle_->solve_ik(
        desired_left_ee_pose_,
        desired_right_ee_pose_,
        &current_joint_pos_,
        &current_joint_vel_,
        nullptr,
        nullptr,
        &l_pos_err,
        &l_rot_err,
        &r_pos_err,
        &r_rot_err,
        &collision
    );
    std::cout << "[IKTrajTracker] both arms"
              << " l_pos_err=" << l_pos_err << " l_rot_err=" << l_rot_err
              << " r_pos_err=" << r_pos_err << " r_rot_err=" << r_rot_err
              << " collision=" << (collision ? "true" : "false") << std::endl;

    error_pose_in_ee_frame_ = current_left_ee_pose_.inverse()*desired_left_ee_pose_;
    error_twist_in_ee_frame_ = pinocchio::log6(error_pose_in_ee_frame_);

    l_error_magnitude = std::sqrt(
        error_twist_in_ee_frame_.linear().squaredNorm() +
        error_twist_in_ee_frame_.angular().squaredNorm()*0.05
    );

    error_pose_in_ee_frame_ = current_right_ee_pose_.inverse()*desired_right_ee_pose_;
    error_twist_in_ee_frame_ = pinocchio::log6(error_pose_in_ee_frame_);

    r_error_magnitude = std::sqrt(
        error_twist_in_ee_frame_.linear().squaredNorm() +
        error_twist_in_ee_frame_.angular().squaredNorm()*0.05
    );

    return result;
}

JointState IKTrajTracker::control_left_arm(
    JointState current_state,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel
)
{
    update(current_state);

    desired_left_ee_pose_ = pinocchio::SE3(desired_ee_pose);

    double l_pos_err = 0.0, l_rot_err = 0.0;
    bool collision = false;
    result = ik_handle_->solve_ik(
        desired_left_ee_pose_,
        true,
        &current_joint_pos_,
        &current_joint_vel_,
        nullptr,
        &l_pos_err,
        &l_rot_err,
        &collision
    );
    std::cout << "[IKTrajTracker] left arm"
              << " pos_err=" << l_pos_err << " rot_err=" << l_rot_err
              << " collision=" << (collision ? "true" : "false") << std::endl;

    {
        JointState filtered;
        filtered.name.reserve(result.name.size());
        filtered.position.reserve(result.position.size());
        filtered.velocity.reserve(result.velocity.size());
        filtered.effort.reserve(result.effort.size());
        for (size_t i = 0; i < result.name.size(); ++i) {
            if (result.name[i].rfind("right_", 0) == 0) continue;
            filtered.name.push_back(result.name[i]);
            if (i < result.position.size()) filtered.position.push_back(result.position[i]);
            if (i < result.velocity.size()) filtered.velocity.push_back(result.velocity[i]);
            if (i < result.effort.size())   filtered.effort.push_back(result.effort[i]);
        }
        result = std::move(filtered);
    }

    error_pose_in_ee_frame_ = current_left_ee_pose_.inverse()*desired_left_ee_pose_;
    error_twist_in_ee_frame_ = pinocchio::log6(error_pose_in_ee_frame_);

    l_error_magnitude = std::sqrt(
        error_twist_in_ee_frame_.linear().squaredNorm() +
        error_twist_in_ee_frame_.angular().squaredNorm()*0.05
    );

    return result;
}

JointState IKTrajTracker::control_right_arm(
    JointState current_state,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel
)
{
    update(current_state);
    
    desired_right_ee_pose_ = pinocchio::SE3(desired_ee_pose);

    double r_pos_err = 0.0, r_rot_err = 0.0;
    bool collision = false;
    result = ik_handle_->solve_ik(
        desired_right_ee_pose_,
        false,
        &current_joint_pos_,
        &current_joint_vel_,
        nullptr,
        &r_pos_err,
        &r_rot_err,
        &collision
    );
    std::cout << "[IKTrajTracker] right arm"
              << " pos_err=" << r_pos_err << " rot_err=" << r_rot_err
              << " collision=" << (collision ? "true" : "false") << std::endl;

    {
        JointState filtered;
        filtered.name.reserve(result.name.size());
        filtered.position.reserve(result.position.size());
        filtered.velocity.reserve(result.velocity.size());
        filtered.effort.reserve(result.effort.size());
        for (size_t i = 0; i < result.name.size(); ++i) {
            if (result.name[i].rfind("left_", 0) == 0) continue;
            filtered.name.push_back(result.name[i]);
            if (i < result.position.size()) filtered.position.push_back(result.position[i]);
            if (i < result.velocity.size()) filtered.velocity.push_back(result.velocity[i]);
            if (i < result.effort.size())   filtered.effort.push_back(result.effort[i]);
        }
        result = std::move(filtered);
    }

    error_pose_in_ee_frame_ = current_right_ee_pose_.inverse()*desired_right_ee_pose_;
    error_twist_in_ee_frame_ = pinocchio::log6(error_pose_in_ee_frame_);

    r_error_magnitude = std::sqrt(
        error_twist_in_ee_frame_.linear().squaredNorm() +
        error_twist_in_ee_frame_.angular().squaredNorm()*0.05
    );

    return result;

}



} // HumanoidPilot namespace