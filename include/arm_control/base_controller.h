#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include "base/base.h"

#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

namespace ArmPilot {

class ArmController {

public:
    ArmController(
        pinocchio::Model& model
    );

    Eigen::MatrixXd get_current_left_ee_pose(){return current_left_ee_pose_.toHomogeneousMatrix();};
    Eigen::MatrixXd get_current_right_ee_pose(){return current_right_ee_pose_.toHomogeneousMatrix();};

    double get_current_left_ee_error(){return l_error_magnitude;};
    double get_current_right_ee_error(){return r_error_magnitude;};

    JointState get_grav_ff(Eigen::VectorXd current_joint_pos);

    std::map<std::string, int> get_joint_idx_map();

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

    Eigen::VectorXd l_control_torques;
    Eigen::VectorXd r_control_torques;
    Eigen::VectorXd grav_torques;
    Eigen::VectorXd torques;

    double dt = 0.01;

    double l_error_magnitude;
    double r_error_magnitude;

}; // ArmControl class

} // ArmPilot namespace

#endif // BASE_CONTROLLER_H