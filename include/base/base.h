#ifndef BASE_H
#define BASE_H

#include <string>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace ArmPilot {

struct RobotConfig {
    std::string asset_file = "../assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
    std::string asset_root = "../assets/g1/";
    int NUM_DOF = 29;
};

class G1DualArm;
class G1_29_ArmIK;
class ImpedanceController;
class EE_MotionPlanner;

class G1DualArm {

public:

    G1DualArm(
        const RobotConfig* robot_config = nullptr
    );

    G1_29_ArmIK* ik;
    ImpedanceController* controller;
    EE_MotionPlanner* motion_planner;

protected:
    void initialize_wrist_joints_to_lock();
    void initialize_leg_joints_to_lock();
    void initialize_palm_joints_to_lock();
    void add_end_effector_frames();

    RobotConfig robot_config_;

    pinocchio::Model robot_model_;
    pinocchio::Model upper_body_;
    pinocchio::Model upper_body_wo_palms;

    pinocchio::GeometryModel geom_robot_model_;
    pinocchio::GeometryModel geom_upper_body_;
    pinocchio::GeometryModel geom_upper_body_wo_palms;

    Eigen::VectorXd reference_config;

    std::vector<pinocchio::JointIndex> leg_joints;
    std::vector<pinocchio::JointIndex> palm_joints;

    std::vector<std::string> mixed_joints_to_lock_ids_;

};


} // ArmPilot namespace

#endif