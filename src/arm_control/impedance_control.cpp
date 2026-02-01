#include "arm_control/impedance_control.h"

#include <pinocchio//algorithm/utils/motion.hpp>

namespace ArmPilot {
    
Eigen::VectorXd ImpedanceController::control_left_arm(
    Eigen::VectorXd current_joint_pos,
    Eigen::VectorXd current_joint_vel,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel
)
{
    current_joint_pos_ = Eigen::VectorXd(current_joint_pos);
    current_joint_vel_ = Eigen::VectorXd(current_joint_vel);

    desired_ee_pose_ = pinocchio::SE3(desired_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_ee_vel);

    update();
    compute_left_arm_control_torques();

    torques = grav_torques + control_torques;

    return torques;
}

Eigen::VectorXd ImpedanceController::control_right_arm(
    Eigen::VectorXd current_joint_pos,
    Eigen::VectorXd current_joint_vel,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel
)
{
    current_joint_pos_ = Eigen::VectorXd(current_joint_pos);
    current_joint_vel_ = Eigen::VectorXd(current_joint_vel);

    desired_ee_pose_ = pinocchio::SE3(desired_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_ee_vel);

    update();
    compute_right_arm_control_torques();

    torques = grav_torques + control_torques;

    return torques;
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

    control_torques += J_body_left_ee_.transpose() * (
        Kp * error_twist_in_ee_frame_ + 
        Kd * error_vel_in_ee_frame_
    ).toVector();
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

    control_torques += J_body_right_ee_.transpose() * (
        Kp * error_twist_in_ee_frame_ + 
        Kd * error_vel_in_ee_frame_
    ).toVector();
}




} // ArmPilot namespace