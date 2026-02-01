#include "arm_control/arm_control.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace ArmPilot {

ArmController::ArmController(
    pinocchio::Model& model
) : model_(model) 
{
    data_ = pinocchio::Data(model_);

    left_ee_id_ = model_.getFrameId("L_ee");
    right_ee_id_ = model_.getFrameId("R_ee");

    last_joint_pos_ = Eigen::VectorXd::Zero(model_.nv);

}

void ArmController::update()
{
    pinocchio::forwardKinematics(
        model_, 
        data_, 
        current_joint_pos_, 
        current_joint_vel_
    );
    
    J_body_left_ee_ = pinocchio::getFrameJacobian(model_, data_, left_ee_id_, pinocchio::LOCAL);
    J_body_right_ee_ = pinocchio::getFrameJacobian(model_, data_, right_ee_id_, pinocchio::LOCAL);

    current_left_ee_pose_ = data_.oMi[left_ee_id_];
    current_right_ee_pose_ = data_.oMi[right_ee_id_];

    current_left_ee_vel_in_ee_frame_ = pinocchio::getFrameVelocity(model_, data_,left_ee_id_);
    current_right_ee_vel_in_ee_frame_ = pinocchio::getFrameVelocity(model_, data_,right_ee_id_);

    control_torques = Eigen::VectorXd::Zero(model_.nv);

    pinocchio::computeGeneralizedGravity(model_, data_, current_joint_pos_);
    grav_torques = data_.g;

    last_joint_pos_ = current_joint_pos_;
}

} // ArmControl namespace