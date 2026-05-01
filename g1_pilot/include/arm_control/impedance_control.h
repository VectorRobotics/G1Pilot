#ifndef IMPD_CTRL_H
#define IMPD_CTRL_H

#include "base_controller.h"

namespace ArmPilot {

class ImpedanceController : public ArmController {
    
public:
    ImpedanceController(
        pinocchio::Model& model
    ) : ArmController(model) { };

    JointState control_both_arms(
    JointState current_state,
    Eigen::Matrix4d desired_l_ee_pose,
    Eigen::Matrix4d desired_r_ee_pose,
    Eigen::VectorXd desired_l_ee_vel = Eigen::VectorXd::Zero(6),
    Eigen::VectorXd desired_r_ee_vel = Eigen::VectorXd::Zero(6)
    );

    JointState control_left_arm(
    JointState current_state,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel = Eigen::VectorXd::Zero(6)
    );

    JointState control_right_arm(
    JointState current_state,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel = Eigen::VectorXd::Zero(6)
    );

    double Kp_linear = 30;
    double Kp_angular = 5;
    double Kd_linear = 0;
    double Kd_angular = 0;
    double Ki_linear = 0;
    double Ki_angular = 0;
    double Ki_max = 5.0; // anti-windup clamp

private:

    void compute_left_arm_control_torques();
    void compute_right_arm_control_torques();

    pinocchio::SE3 desired_ee_pose_;
    pinocchio::Motion desired_ee_vel_;
    pinocchio::Motion desired_ee_vel_in_ee_frame_;

    pinocchio::SE3 error_pose_in_ee_frame_;
    pinocchio::Motion error_twist_in_ee_frame_;

    pinocchio::Motion error_vel_in_ee_frame_;

    Eigen::VectorXd torques_dual_arm_;

    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Ki = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd error_integral_in_ee_frame_ = Eigen::VectorXd::Zero(6);

}; // ImpedanceController class
} // ArmControl namespace

#endif