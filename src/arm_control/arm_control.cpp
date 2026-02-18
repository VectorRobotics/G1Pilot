#include "arm_control/base_controller.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>
namespace ArmPilot {

ArmController::ArmController(
    pinocchio::Model& model
) : model_(model) 
{
    data_ = pinocchio::Data(model_);

    left_ee_id_ = model_.getFrameId("L_ee");
    right_ee_id_ = model_.getFrameId("R_ee");

    last_joint_pos_ = Eigen::VectorXd::Zero(model_.nq);

}

void ArmController::update()
{
    pinocchio::forwardKinematics(
        model_,
        data_,
        current_joint_pos_,
        current_joint_vel_
    );
    pinocchio::computeJointJacobians(model_, data_);
    pinocchio::updateFramePlacements(model_, data_);

    J_body_left_ee_ = pinocchio::getFrameJacobian(model_, data_, left_ee_id_, pinocchio::LOCAL);
    J_body_right_ee_ = pinocchio::getFrameJacobian(model_, data_, right_ee_id_, pinocchio::LOCAL);

    current_left_ee_pose_ = data_.oMf[left_ee_id_];
    current_right_ee_pose_ = data_.oMf[right_ee_id_];

    current_left_ee_vel_in_ee_frame_ = pinocchio::getFrameVelocity(model_, data_,left_ee_id_);
    current_right_ee_vel_in_ee_frame_ = pinocchio::getFrameVelocity(model_, data_,right_ee_id_);

    l_control_torques = Eigen::VectorXd::Zero(model_.nv);
    r_control_torques = Eigen::VectorXd::Zero(model_.nv);
    torques = Eigen::VectorXd::Zero(model_.nv);

    pinocchio::computeGeneralizedGravity(model_, data_, current_joint_pos_);
    grav_torques = 1.052*data_.g;

    last_joint_pos_ = current_joint_pos_;
}

} // ArmControl namespace