#include "arm_mp/joint_space_planner.h"
#include "base/humanoid.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace HumanoidPilot {

JointSpacePlanner::JointSpacePlanner(
    pinocchio::Model& model,
    HumanoidIK* ik_handle,
    bool left_arm,
    double step_size,
    double goal_bias,
    double rewire_factor,
    double solve_time_seconds,
    double goal_tolerance,
    double validity_resolution
) :
    model_(model),
    data_(model_),
    ik_handle_(ik_handle),
    left_arm_(left_arm),
    nq_(model.nq),
    left_ee_id_(model_.getFrameId("L_ee")),
    right_ee_id_(model_.getFrameId("R_ee")),
    step_size_(step_size),
    goal_bias_(goal_bias),
    rewire_factor_(rewire_factor),
    solve_time_seconds_(solve_time_seconds),
    goal_tolerance_(goal_tolerance),
    validity_resolution_(validity_resolution)
{
    auto rv_space = std::make_shared<ob::RealVectorStateSpace>(nq_);
    ob::RealVectorBounds bounds(nq_);
    for (int i = 0; i < nq_; ++i) {
        bounds.setLow(i, model_.lowerPositionLimit(i));
        bounds.setHigh(i, model_.upperPositionLimit(i));
    }
    rv_space->setBounds(bounds);
    ompl_space_ = rv_space;

    ompl_si_ = std::make_shared<ob::SpaceInformation>(ompl_space_);
    ompl_si_->setStateValidityChecker([this](const ob::State* s) {
        const auto* rv = s->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(nq_);
        for (int i = 0; i < nq_; ++i) q(i) = rv->values[i];
        if (ik_handle_ == nullptr) return true;
        return !ik_handle_->check_collision(q);
        // return true;
    });
    ompl_si_->setStateValidityCheckingResolution(validity_resolution_);
    ompl_si_->setup();

    planner_ = std::make_shared<og::RRTstar>(ompl_si_);
    planner_->setRange(step_size_);
    planner_->setGoalBias(goal_bias_);
    planner_->setRewireFactor(rewire_factor_);
}

JointSpacePlanner::~JointSpacePlanner() {}

Eigen::VectorXd JointSpacePlanner::poseToJointConfig(const Eigen::MatrixXd& pose) {
    if (pose.rows() != 4 || pose.cols() != 4) {
        throw std::invalid_argument("JointSpacePlanner: pose must be 4x4 SE(3)");
    }
    if (ik_handle_ == nullptr) {
        throw std::runtime_error("JointSpacePlanner: ik_handle is null");
    }

    double pos_err = 0.0;
    double rot_err = 0.0;
    JointState js = ik_handle_->solve_ik(
        pose, left_arm_,
        nullptr, nullptr, nullptr,
        &pos_err, &rot_err);

    auto [q, v, e] = jointstate_to_vectors(js, model_);
    (void)v; (void)e;

    std::cout << "[poseToJointConfig] IK pos err (m): " << pos_err
              << ", rot err (rad): " << rot_err << std::endl;

    return q;
}

std::vector<Eigen::VectorXd> JointSpacePlanner::runRRTStar(
    const Eigen::VectorXd& start_q,
    const Eigen::VectorXd& goal_q
) {
    ob::ScopedState<ob::RealVectorStateSpace> start(ompl_space_);
    ob::ScopedState<ob::RealVectorStateSpace> goal(ompl_space_);
    for (int i = 0; i < nq_; ++i) {
        start[i] = start_q(i);
        goal[i] = goal_q(i);
    }

    if (!ompl_si_->isValid(start.get())) {
        std::cout << "[runRRTStar] start state is in collision" << std::endl;
        return {};
    }
    if (!ompl_si_->isValid(goal.get())) {
        std::cout << "[runRRTStar] goal state is in collision" << std::endl;
        return {};
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(ompl_si_);
    pdef->setStartAndGoalStates(start, goal, goal_tolerance_);
    pdef->setOptimizationObjective(
        std::make_shared<ob::PathLengthOptimizationObjective>(ompl_si_));

    planner_->clear();
    planner_->setProblemDefinition(pdef);

    ob::PlannerStatus status = planner_->solve(
        ob::timedPlannerTerminationCondition(solve_time_seconds_));
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cout << "[runRRTStar] no exact solution (status="
                  << status.asString() << ")" << std::endl;
        return {};
    }

    auto path_geom = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
    if (!path_geom) {
        return {};
    }

    og::PathSimplifier simplifier(ompl_si_);
    simplifier.ropeShortcutPath(*path_geom);
    simplifier.smoothBSpline(*path_geom);

    std::vector<Eigen::VectorXd> out;
    out.reserve(path_geom->getStateCount());
    for (std::size_t i = 0; i < path_geom->getStateCount(); ++i) {
        const auto* rv = path_geom->getState(i)->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(nq_);
        for (int j = 0; j < nq_; ++j) q(j) = rv->values[j];
        out.push_back(std::move(q));
    }
    return out;
}

Eigen::Matrix4d JointSpacePlanner::configToPose(const Eigen::VectorXd& q) {
    pinocchio::framesForwardKinematics(model_, data_, q);

    return left_arm_
    ? data_.oMf[left_ee_id_].toHomogeneousMatrix()
    : data_.oMf[right_ee_id_].toHomogeneousMatrix();
}

std::vector<Eigen::MatrixXd> JointSpacePlanner::configsToPoses(
    const std::vector<Eigen::VectorXd>& path
) {
    std::vector<Eigen::MatrixXd> cart_space;
    cart_space.reserve(path.size());
    for (const auto& q : path) cart_space.push_back(configToPose(q));
    return cart_space;
}

std::vector<Eigen::VectorXd> JointSpacePlanner::resamplePath(
    const std::vector<Eigen::VectorXd>& waypoints, int steps
) {
    if (waypoints.empty()) return {};
    if (steps <= 1 || waypoints.size() == 1) return waypoints;

    const int N = static_cast<int>(waypoints.size());
    std::vector<double> seg_len(N - 1);
    double total = 0.0;
    for (int i = 0; i < N - 1; ++i) {
        seg_len[i] = (waypoints[i + 1] - waypoints[i]).norm();
        total += seg_len[i];
    }
    if (total <= 0.0) return {waypoints.front()};

    std::vector<Eigen::VectorXd> out(steps);
    for (int k = 0; k < steps; ++k) {
        double s = (static_cast<double>(k) / (steps - 1)) * total;
        double acc = 0.0;
        int i = 0;
        for (; i + 1 < N; ++i) {
            if (acc + seg_len[i] >= s) break;
            acc += seg_len[i];
        }
        if (i + 1 >= N) {
            out[k] = waypoints.back();
            continue;
        }
        double t = (seg_len[i] > 0.0) ? (s - acc) / seg_len[i] : 0.0;
        out[k] = waypoints[i] + t * (waypoints[i + 1] - waypoints[i]);
    }
    return out;
}

std::pair<
std::vector<Eigen::VectorXd>,
std::vector<Eigen::MatrixXd>>
JointSpacePlanner::planPath(
    const Eigen::MatrixXd* goal_pose,
    const Eigen::MatrixXd* start_pose,
    const Eigen::VectorXd* /*start_vel*/,
    const Eigen::VectorXd* /*goal_vel*/,
    const Eigen::VectorXd* /*start_acc*/,
    const Eigen::VectorXd* /*goal_acc*/
) {
    if (goal_pose == nullptr) {
        throw std::invalid_argument("JointSpacePlanner::planPath: goal_pose is null");
    }

    if (start_pose == nullptr) {
        throw std::invalid_argument("JointSpacePlanner::planPath: start_pose is null");
    }

    std::cout << "IK for start pose:" << std::endl;
    start_q_ = poseToJointConfig(*start_pose);
    if (start_q_.size() == 0)
        return {};

    std::cout << "IK for goal pose:" << std::endl;
    goal_q_ = poseToJointConfig(*goal_pose);
    if (goal_q_.size() == 0)
        return {};

    waypoints_ = runRRTStar(start_q_, goal_q_);

    return {waypoints_, configsToPoses(waypoints_)};
}

std::pair<
std::vector<Eigen::VectorXd>,
std::vector<Eigen::MatrixXd>>
JointSpacePlanner::planTrajectory(
    const Eigen::MatrixXd* goal_pose,
    const Eigen::MatrixXd* start_pose,
    const Eigen::VectorXd* start_vel,
    const Eigen::VectorXd* goal_vel,
    const Eigen::VectorXd* start_acc,
    const Eigen::VectorXd* goal_acc,
    double time_step,
    const double MAX_LIN_VEL,
    const double /*MAX_ANG_VEL*/,
    const double /*MAX_LIN_ACC*/,
    const double /*MAX_ANG_ACC*/
) {
    if (goal_pose == nullptr) {
        throw std::invalid_argument("JointSpacePlanner::planTrajectory: goal_pose is null");
    }

    Eigen::Vector3d p_start = (start_pose != nullptr)
        ? Eigen::Vector3d(start_pose->block<3,1>(0,3))
        : Eigen::Vector3d::Zero();
    double task_dist = (goal_pose->block<3,1>(0,3) - p_start).norm();
    double vel_cap = std::max(MAX_LIN_VEL, 1e-6);
    double duration = task_dist / vel_cap;
    int steps = std::max(2,
        static_cast<int>(std::ceil(duration / std::max(time_step, 1e-6))));

    (void) planPath(goal_pose, start_pose,
                           start_vel, goal_vel, start_acc, goal_acc);

    std::vector<Eigen::VectorXd> path = resamplePath(waypoints_, steps);
    return {path, configsToPoses(path)};

    // return planPath(goal_pose, start_pose,
    //                        start_vel, goal_vel, start_acc, goal_acc);
}

} // namespace HumanoidPilot
