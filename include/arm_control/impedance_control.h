#include "arm_control/base_controller.h"

namespace ArmPilot {

class ImpedanceController : public ArmController {
    
public:
    ImpedanceController(
        pinocchio::Model& model
    ) : ArmController(model) {};

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

private:

    void compute_left_arm_control_torques();
    void compute_right_arm_control_torques();

    const double Kp = 3.0;
    const double Kd = 0.1;

    pinocchio::SE3 desired_ee_pose_;
    pinocchio::Motion desired_ee_vel_;
    pinocchio::Motion desired_ee_vel_in_ee_frame_;

    pinocchio::SE3 error_pose_in_ee_frame_;
    pinocchio::Motion error_twist_in_ee_frame_;

    pinocchio::Motion error_vel_in_ee_frame_;

    Eigen::VectorXd torques_dual_arm_;

}; // ImpedanceController class
} // ArmControl namespace