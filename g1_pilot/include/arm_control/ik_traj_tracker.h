#ifndef IK_TRAJ_H
#define IK_TRAJ_H

#include "base_controller.h"
#include "../arm_ik/arm_ik.h"

namespace HumanoidPilot {

class IKTrajTracker : public ArmController {
    
public:
    IKTrajTracker(
        pinocchio::Model& model,
        HumanoidIK* ik_handle
    ) : ArmController(model), ik_handle_(ik_handle){ };

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

    JointState result;

private:

    pinocchio::SE3 desired_left_ee_pose_;
    pinocchio::SE3 desired_right_ee_pose_;

    pinocchio::SE3 error_pose_in_ee_frame_;
    pinocchio::Motion error_twist_in_ee_frame_;
   
    HumanoidIK* ik_handle_;

}; // IKTrajTracker class
} // ArmControl namespace

#endif