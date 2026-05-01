#ifndef HUMANOID_H
#define HUMANOID_H

#include "interfaces.h"

#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/data.hpp>

namespace HumanoidPilot {

// Forward Declarations
class HumanoidIK;
class ImpedanceController;
class PolynomialTrajectoryGenerator;
class VisualServoPlanner;
class IKTrajTracker;
class JointSpacePlanner;

class Humanoid {

public:

    Humanoid(
        const RobotConfig* robot_config = nullptr
    );

    HumanoidIK* ik;
    IKTrajTracker* controller;
    JointSpacePlanner* motion_planner;

    JointState grav_ff(JointState input);

protected:
    struct EEFrame {
        std::string parent_joint;
        Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    };

    void extract_config();
    void add_end_effector_frames();
    void reduce_model(
        std::vector<std::string> joint_names,
        pinocchio::Model &model_in,
        pinocchio::GeometryModel &gmodel_in,
        pinocchio::Model &model_out,
        pinocchio::GeometryModel &gmodel_out
    );

    RobotConfig robot_config_;

    pinocchio::Model model_full;
    pinocchio::Model model;

    pinocchio::GeometryModel geom_full;
    pinocchio::GeometryModel geom;

    pinocchio::Data data;

    EEFrame left_ee_;
    EEFrame right_ee_;

    std::vector<std::string> locked_joints_;

    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd e;
    
    Eigen::VectorXd grav_torques;

};


} // HumanoidPilot namespace

#endif