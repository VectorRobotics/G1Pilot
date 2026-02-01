#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "base/base.h"

#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

namespace ArmPilot {

class ArmController {

public:
    ArmController(
        pinocchio::Model& model
    );

protected:
    void update();
    pinocchio::Model model_;
    pinocchio::Data data_;

    pinocchio::JointIndex left_ee_id_;
    pinocchio::JointIndex right_ee_id_;

    Eigen::VectorXd current_joint_pos_;
    Eigen::VectorXd last_joint_pos_;
    Eigen::VectorXd current_joint_vel_;

    pinocchio::SE3 current_left_ee_pose_;
    pinocchio::SE3 current_right_ee_pose_;

    pinocchio::Motion current_left_ee_vel_in_ee_frame_;
    pinocchio::Motion current_right_ee_vel_in_ee_frame_;

    pinocchio::Data::Matrix6x J_body_left_ee_;
    pinocchio::Data::Matrix6x J_body_right_ee_;

    Eigen::VectorXd control_torques;
    Eigen::VectorXd grav_torques;
    Eigen::VectorXd torques;

    double dt = 0.01;

}; // ArmControl class

} // ArmPilot namespace

#endif // ARM_CONTROL_H