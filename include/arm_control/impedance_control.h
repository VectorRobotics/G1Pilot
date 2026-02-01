#include "arm_control/arm_control.h"

namespace ArmPilot {

class ImpedanceController : public ArmController {
    
public:
    ImpedanceController(
        pinocchio::Model& model
    ) : ArmController(model) {};

    Eigen::VectorXd control_left_arm(
    Eigen::VectorXd current_joint_pos,
    Eigen::VectorXd current_joint_vel,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel = Eigen::VectorXd::Zero(6)
    );

    Eigen::VectorXd control_left_arm(
    Eigen::VectorXd current_joint_pos,
    Eigen::Matrix4d desired_ee_pose
    ){
        return control_left_arm(
            current_joint_pos,
            (current_joint_pos - last_joint_pos_)/dt,
            desired_ee_pose
        );
    };

    Eigen::VectorXd control_right_arm(
    Eigen::VectorXd current_joint_pos,
    Eigen::VectorXd current_joint_vel,
    Eigen::Matrix4d desired_ee_pose,
    Eigen::VectorXd desired_ee_vel = Eigen::VectorXd::Zero(6)
    );

    Eigen::VectorXd control_right_arm(
    Eigen::VectorXd current_joint_pos,
    Eigen::Matrix4d desired_ee_pose
    ){
        return control_right_arm(
            current_joint_pos,
            (current_joint_pos - last_joint_pos_)/dt,
            desired_ee_pose
        );
    };

private:

    void compute_left_arm_control_torques();
    void compute_right_arm_control_torques();

    const double Kp = 1000.0;
    const double Kd = 1000.0;

    pinocchio::SE3 desired_ee_pose_;
    pinocchio::Motion desired_ee_vel_;
    pinocchio::Motion desired_ee_vel_in_ee_frame_;

    pinocchio::SE3 error_pose_in_ee_frame_;
    pinocchio::Motion error_twist_in_ee_frame_;

    pinocchio::Motion error_vel_in_ee_frame_;

}; // ImpedanceController class
} // ArmControl namespace