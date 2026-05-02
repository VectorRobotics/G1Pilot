#include "arm_control/impedance_control.h"

#include <pinocchio/algorithm/utils/motion.hpp>
#include <Eigen/SVD>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace HumanoidPilot {

namespace {

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& M, double damping = 1e-5) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.matrixV() * (

            svd.singularValues().array() /
            (svd.singularValues().array().square() + damping*damping)
            
        ).matrix().asDiagonal() * svd.matrixU().transpose();
}

void saturateTorqueRate(Eigen::VectorXd& tau_new,
                        const Eigen::VectorXd& tau_prev,
                        double delta_max) {
    tau_new = tau_prev + (tau_new - tau_prev).cwiseMax(-delta_max).cwiseMin(delta_max);
}

} // anonymous namespace

void ImpedanceController::set_nullspace_target(const Eigen::VectorXd& q_d) {
    if (q_d.size() != q_nullspace_desired_.size()) {
        throw std::invalid_argument(
            "[ImpedanceController] nullspace target size mismatch"
        );
    }
    q_nullspace_desired_ = q_d;
}

void ImpedanceController::reset() {
    prev_torques_.setZero();
    l_error_magnitude = 0.0;
    r_error_magnitude = 0.0;
}

void ImpedanceController::compute_nullspace_torques(const Eigen::MatrixXd& J) {
    tau0_ = Kp_nullspace * (q_nullspace_desired_ - current_joint_pos_)
          - Kd_nullspace * current_joint_vel_;

    nullspace_torques_ = (
        tau0_ - 
        J.transpose() * (pseudoInverse(J.transpose()) * tau0_)
    );
}

JointState ImpedanceController::control_both_arms(
    JointState current_state,
    Eigen::Matrix4d desired_l_ee_pose,
    Eigen::Matrix4d desired_r_ee_pose,
    Eigen::VectorXd desired_l_ee_vel,
    Eigen::VectorXd desired_r_ee_vel
)
{
    update(current_state);

    Kp.head<3>().setConstant(Kp_linear);
    Kp.tail<3>().setConstant(Kp_angular);
    Kd.head<3>().setConstant(Kd_linear);
    Kd.tail<3>().setConstant(Kd_angular);

    desired_ee_pose_ = pinocchio::SE3(desired_l_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_l_ee_vel);
    compute_left_arm_control_torques();

    desired_ee_pose_ = pinocchio::SE3(desired_r_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_r_ee_vel);
    compute_right_arm_control_torques();

    J_dual_.topRows(6)    = J_left_ee_;
    J_dual_.bottomRows(6) = J_right_ee_;
    compute_nullspace_torques(J_dual_);

    torques = grav_torques + l_control_torques + r_control_torques + nullspace_torques_;

    saturateTorqueRate(torques, prev_torques_, max_torque_rate / update_frequency);
    prev_torques_ = torques;

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
    update(current_state);
    
    Kp.head<3>().setConstant(Kp_linear);
    Kp.tail<3>().setConstant(Kp_angular);
    Kd.head<3>().setConstant(Kd_linear);
    Kd.tail<3>().setConstant(Kd_angular);
    
    desired_ee_pose_ = pinocchio::SE3(desired_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_ee_vel);
    compute_left_arm_control_torques();

    compute_nullspace_torques(J_left_ee_);

    torques = grav_torques + l_control_torques + nullspace_torques_;

    saturateTorqueRate(torques, prev_torques_, max_torque_rate / update_frequency);
    prev_torques_ = torques;

    JointState result = vectors_to_jointstate(
        current_joint_pos_,
        current_joint_vel_,
        torques,
        model_
    );

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

    return result;
}

JointState ImpedanceController::control_right_arm(
    JointState current_state,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel
)
{
    update(current_state);
    
    Kp.head<3>().setConstant(Kp_linear);
    Kp.tail<3>().setConstant(Kp_angular);
    Kd.head<3>().setConstant(Kd_linear);
    Kd.tail<3>().setConstant(Kd_angular);
    
    desired_ee_pose_ = pinocchio::SE3(desired_ee_pose);
    desired_ee_vel_ = pinocchio::Motion(desired_ee_vel);
    compute_right_arm_control_torques();

    compute_nullspace_torques(J_right_ee_);

    torques = grav_torques + r_control_torques + nullspace_torques_;

    saturateTorqueRate(torques, prev_torques_, max_torque_rate / update_frequency);
    prev_torques_ = torques;

    JointState result = vectors_to_jointstate(
        current_joint_pos_,
        current_joint_vel_,
        torques,
        model_
    );

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

    return result;

}

void ImpedanceController::compute_left_arm_control_torques(){

    error_twist_in_world_.linear() =
        desired_ee_pose_.translation() - current_left_ee_pose_.translation();
    error_twist_in_world_.angular() = pinocchio::log3(
        desired_ee_pose_.rotation() * current_left_ee_pose_.rotation().transpose()
    );

    error_vel_in_world_ = desired_ee_vel_ - current_left_ee_vel_in_world_;

    l_control_torques = J_left_ee_.transpose() * (
        Kp.asDiagonal() * error_twist_in_world_.toVector() +
        Kd.asDiagonal() * error_vel_in_world_.toVector()
    );

    l_error_magnitude = std::sqrt(
        error_twist_in_world_.linear().squaredNorm() +
        error_twist_in_world_.angular().squaredNorm()*0.05
    );

    std::cout<< "lin: " << error_twist_in_world_.linear().squaredNorm() <<std::endl;
    std::cout<< "ang: " << error_twist_in_world_.angular().squaredNorm() <<std::endl;
    std::cout<< "mag: " << l_error_magnitude <<std::endl;
}

void ImpedanceController::compute_right_arm_control_torques(){

    error_twist_in_world_.linear() =
        desired_ee_pose_.translation() - current_right_ee_pose_.translation();
    error_twist_in_world_.angular() = pinocchio::log3(
        desired_ee_pose_.rotation() * current_right_ee_pose_.rotation().transpose()
    );

    error_vel_in_world_ = desired_ee_vel_ - current_right_ee_vel_in_world_;

    r_control_torques = J_right_ee_.transpose() * (
        Kp.asDiagonal() * error_twist_in_world_.toVector() +
        Kd.asDiagonal() * error_vel_in_world_.toVector()
    );

    r_error_magnitude = std::sqrt(
        error_twist_in_world_.linear().squaredNorm() +
        error_twist_in_world_.angular().squaredNorm()*0.05
    );
    std::cout<< "lin: " << error_twist_in_world_.linear().squaredNorm() <<std::endl;
    std::cout<< "ang: " << error_twist_in_world_.angular().squaredNorm() <<std::endl;
}




} // HumanoidPilot namespace