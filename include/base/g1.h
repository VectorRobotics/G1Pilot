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

class G1DualArm {

public:

    G1DualArm(
        const RobotConfig* robot_config = nullptr
    );

    HumanoidIK* ik;
    ImpedanceController* controller;
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

    std::vector<std::string> waist_joints;
    std::vector<std::string> leg_joints;
    std::vector<std::string> hand_joints;
    std::vector<std::string> wrist_joints;

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