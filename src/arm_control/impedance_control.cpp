#include "arm_control/impedance_control.h"

#include <pinocchio/algorithm/utils/motion.hpp>
#include <iostream>

namespace ArmPilot {
    
JointState ImpedanceController::control_both_arms(
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

    desired_ee_pose_ = pinocchio::SE3(desired_l_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_l_ee_vel);
    compute_left_arm_control_torques();

    desired_ee_pose_ = pinocchio::SE3(desired_r_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_r_ee_vel);
    compute_right_arm_control_torques();

    torques = grav_torques + l_control_torques + r_control_torques;

    JointState result = vectors_to_jointstate(
        current_joint_pos_,
        current_joint_vel_,
        torques,
        model_
    );

    return result;
}

JointState ImpedanceController::control_left_arm(
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
    
    desired_ee_pose_ = pinocchio::SE3(desired_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_ee_vel);
    compute_left_arm_control_torques();

    torques = grav_torques + l_control_torques;

    JointState result = vectors_to_jointstate(
        current_joint_pos_,
        current_joint_vel_,
        torques,
        model_
    );

    return result;
}

JointState ImpedanceController::control_right_arm(
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
    
    desired_ee_pose_ = pinocchio::SE3(desired_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_ee_vel);
    compute_right_arm_control_torques();

    torques = grav_torques + r_control_torques;

    JointState result = vectors_to_jointstate(
        current_joint_pos_,
        current_joint_vel_,
        torques,
        model_
    );

    return result;

}

void ImpedanceController::compute_left_arm_control_torques(){

    error_pose_in_ee_frame_ = current_left_ee_pose_.inverse()*desired_ee_pose_;
    error_twist_in_ee_frame_ = pinocchio::log6(error_pose_in_ee_frame_); 

    desired_ee_vel_in_ee_frame_ = pinocchio::changeReferenceFrame(
        current_left_ee_pose_,
        desired_ee_vel_,
        pinocchio::WORLD,
        pinocchio::LOCAL
    );

    error_vel_in_ee_frame_ = desired_ee_vel_in_ee_frame_ - current_left_ee_vel_in_ee_frame_;

    l_control_torques = J_body_left_ee_.transpose() * (
        Kp * error_twist_in_ee_frame_ + 
        Kd * error_vel_in_ee_frame_
    ).toVector();

    l_error_magnitude = std::sqrt(
        error_twist_in_ee_frame_.linear().squaredNorm() + 
        error_twist_in_ee_frame_.angular().squaredNorm()*0.09
    );
}

void ImpedanceController::compute_right_arm_control_torques(){

    error_pose_in_ee_frame_ = current_right_ee_pose_.inverse()*desired_ee_pose_;
    error_twist_in_ee_frame_ = pinocchio::log6(error_pose_in_ee_frame_); 

    desired_ee_vel_in_ee_frame_ = pinocchio::changeReferenceFrame(
        current_right_ee_pose_,
        desired_ee_vel_,
        pinocchio::WORLD,
        pinocchio::LOCAL
    );

    error_vel_in_ee_frame_ = desired_ee_vel_in_ee_frame_ - current_right_ee_vel_in_ee_frame_;

    r_control_torques = J_body_right_ee_.transpose() * (
        Kp * error_twist_in_ee_frame_ + 
        Kd * error_vel_in_ee_frame_
    ).toVector();

    r_error_magnitude = std::sqrt(
        error_twist_in_ee_frame_.linear().squaredNorm() + 
        error_twist_in_ee_frame_.angular().squaredNorm()*0.09
    );
}




} // ArmPilot namespace