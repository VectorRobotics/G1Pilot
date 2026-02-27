#include "arm_control/ik_traj_tracker.h"

#include <pinocchio/algorithm/utils/motion.hpp>
#include <iostream>

namespace ArmPilot {
    
JointState IKTrajTracker::control_both_arms(
    JointState current_state,
    Eigen::Matrix4d desired_l_ee_pose,
    Eigen::Matrix4d desired_r_ee_pose,
    Eigen::VectorXd desired_l_ee_vel,
    Eigen::VectorXd desired_r_ee_vel
)
{
    std::tie(
        current_joint_pos_,
        current_joint_vel_,
        current_joint_eff_
    ) = jointstate_to_vectors(current_state, model_);

    update();
    
    desired_left_ee_pose_ = pinocchio::SE3(desired_l_ee_pose);
    desired_right_ee_pose_ = pinocchio::SE3(desired_r_ee_pose);

    result = ik_handle_->solve_ik(
        desired_left_ee_pose_,
        desired_right_ee_pose_,
        &current_joint_pos_,
        &current_joint_vel_
    );

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
    std::tie(
        current_joint_pos_,
        current_joint_vel_,
        current_joint_eff_
    ) = jointstate_to_vectors(current_state, model_);

    update();

    desired_left_ee_pose_ = pinocchio::SE3(desired_ee_pose);

    result = ik_handle_->solve_ik(
        desired_left_ee_pose_,
        true,
        &current_joint_pos_,
        &current_joint_vel_
    );

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
    std::tie(
        current_joint_pos_,
        current_joint_vel_,
        current_joint_eff_
    ) = jointstate_to_vectors(current_state, model_);

    update();

    desired_right_ee_pose_ = pinocchio::SE3(desired_ee_pose);

    result = ik_handle_->solve_ik(
        desired_right_ee_pose_,
        false,
        &current_joint_pos_,
        &current_joint_vel_
    );

    error_pose_in_ee_frame_ = current_right_ee_pose_.inverse()*desired_right_ee_pose_;
    error_twist_in_ee_frame_ = pinocchio::log6(error_pose_in_ee_frame_);

    r_error_magnitude = std::sqrt(
        error_twist_in_ee_frame_.linear().squaredNorm() +
        error_twist_in_ee_frame_.angular().squaredNorm()*0.05
    );

    return result;

}



} // ArmPilot namespace