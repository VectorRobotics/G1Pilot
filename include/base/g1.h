#ifndef BASE_H
#define BASE_H

#include "interfaces.h"

#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/data.hpp>

namespace ArmPilot {

// Forward Declarations
class HumanoidIK;
class ImpedanceController;
class PolynomialTrajectoryGenerator;
class VisualServoPlanner;
class IKTrajTracker;

class G1DualArm {

public:

    G1DualArm(
        const RobotConfig* robot_config = nullptr
    );

    HumanoidIK* ik;
    IKTrajTracker* controller;
    PolynomialTrajectoryGenerator* motion_planner;

    JointState grav_ff(JointState input);

protected:
    void categorize_joints();
    void add_end_effector_frames();
    void reduce_model(
        std::vector<std::string> joint_names,
        pinocchio::Model &model_in,
        pinocchio::GeometryModel &gmodel_in,
        pinocchio::Model &model_out,
        pinocchio::GeometryModel &gmodel_out
    );

    RobotConfig robot_config_;

    std::vector<std::string> 
        left_leg_joints,
        right_leg_joints,

        waist_joints,

        left_upper_arm_joints,
        right_upper_arm_joints,

        left_wrist_joints,
        right_wrist_joints,

        left_hand_joints,
        right_hand_joints,
        
        leg_joints,
        upper_arm_joints,
        wrist_joints,
        hand_joints,

        left_arm_joints,
        right_arm_joints;

    pinocchio::Model model_g1_29;
    pinocchio::Model model_g1_26;
    pinocchio::Model model_g1_14;
    pinocchio::Model model_g1_14_wo_hands;

    pinocchio::GeometryModel geom_g1_29;
    pinocchio::GeometryModel geom_g1_26;
    pinocchio::GeometryModel geom_g1_14;
    pinocchio::GeometryModel geom_g1_14_wo_hands;

    pinocchio::Data data_g1_26;

    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd e;
    
    Eigen::VectorXd grav_torques;

};


} // ArmPilot namespace

#endif