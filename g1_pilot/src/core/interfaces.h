#ifndef INTERFACES_H
#define INTERFACES_H

#include <g1_pilot/g1_pilot.h>
#include <pinocchio/multibody/model.hpp>

namespace HumanoidPilot{

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