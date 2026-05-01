#ifndef INTERFACES_H
#define INTERFACES_H

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>

namespace HumanoidPilot{

struct JointState {
    std::vector<std::string> name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};

struct RobotConfig {
    std::string asset_file = "../assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
    std::string asset_root = "../assets/g1/";
    std::string config_file = "../config/g1.yaml";
    int NUM_DOF = 29;
};

std::tuple<
Eigen::VectorXd,
Eigen::VectorXd,
Eigen::VectorXd>
jointstate_to_vectors(JointState input, const pinocchio::Model& model);

JointState vectors_to_jointstate(
    Eigen::VectorXd q,
    Eigen::VectorXd v,
    Eigen::VectorXd e,
    const pinocchio::Model& model
);

}

#endif