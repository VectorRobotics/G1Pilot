#ifndef IMPD_CTRL_H
#define IMPD_CTRL_H

#include "base_controller.h"

namespace HumanoidPilot {

class ImpedanceController : public ArmController {

public:
    ImpedanceController(pinocchio::Model& model)
      : ArmController(model),
        q_nullspace_desired_(Eigen::VectorXd::Zero(model.nv)),
        prev_torques_(Eigen::VectorXd::Zero(model.nv)),
        J_dual_(Eigen::MatrixXd::Zero(12, model.nv)),
        nullspace_torques_(Eigen::VectorXd::Zero(model.nv)),
        tau0_(Eigen::VectorXd::Zero(model.nv))
    { }

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

    void set_nullspace_target(const Eigen::VectorXd& q_d);
    void reset();

    double Kp_linear  = 50;
    double Kp_angular = 5;
    double Kd_linear  = 5;
    double Kd_angular = 2;

    // Null-space joint impedance (off by default; pure null-space damping).
    double Kp_nullspace = 25.0;
    double Kd_nullspace = 5.0;

    // Torque rate limit. Per-tick cap = max_torque_rate / update_frequency.
    // Bump update_frequency to match the actual control-loop rate.
    double max_torque_rate  = 50.0;  // N·m / s
    double update_frequency = 100.0; // Hz

private:

    void compute_left_arm_control_torques();
    void compute_right_arm_control_torques();

    void compute_nullspace_torques(const Eigen::MatrixXd& J);

    pinocchio::SE3 desired_ee_pose_;
    pinocchio::Motion desired_ee_vel_;

    pinocchio::Motion error_twist_in_world_;
    pinocchio::Motion error_vel_in_world_;

    Eigen::VectorXd torques_dual_arm_;

    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd q_nullspace_desired_;
    Eigen::VectorXd prev_torques_;

    Eigen::MatrixXd J_dual_;
    Eigen::VectorXd nullspace_torques_;
    Eigen::VectorXd tau0_;

}; // ImpedanceController class
} // ArmControl namespace

#endif