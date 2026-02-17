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

    last_joint_pos_ = Eigen::VectorXd::Zero(model_.nv);

}

std::map<std::string, int> ArmController::get_joint_idx_map(){
        std::map<std::string, int> mapping;
        for (int joint_id = 1; joint_id < model_.njoints; ++joint_id) {
            mapping[model_.names[joint_id]] = model_.joints[joint_id].idx_v();
        }
        return mapping;
    };


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

    get_grav_ff(current_joint_pos_);

    last_joint_pos_ = current_joint_pos_;
}

void ArmController::compute_fk(Eigen::VectorXd current_joint_pos){
    current_joint_pos_ = current_joint_pos;
    pinocchio::forwardKinematics(model_, data_, current_joint_pos_);
    pinocchio::updateFramePlacements(model_, data_);
    current_left_ee_pose_ = data_.oMf[left_ee_id_];
    current_right_ee_pose_ = data_.oMf[right_ee_id_];
}

JointState ArmController::get_grav_ff(Eigen::VectorXd current_joint_pos){
        pinocchio::computeGeneralizedGravity(model_, data_, current_joint_pos);
        grav_torques = data_.g;

        JointState result;

        for (int joint_id = 1; joint_id < model_.njoints; ++joint_id) {
            result.name.push_back(model_.names[joint_id]);
            result.position.push_back(current_joint_pos[model_.joints[joint_id].idx_v()]);
            result.velocity.push_back(0);
            result.effort.push_back(
                grav_torques[ model_.joints[joint_id].idx_v()]
            );
        }

        return result;

}

} // ArmControl namespace