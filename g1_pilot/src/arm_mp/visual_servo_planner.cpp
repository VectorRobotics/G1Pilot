#include "arm_mp/visual_servo_planner.h"

#include <iostream>
#include <stdexcept>

namespace HumanoidPilot{

VisualServoPlanner::VisualServoPlanner(
    pinocchio::Model& model,
    HumanoidIK* ik_handle,
    bool left_arm,
    double approach_offset,
    int final_leg_steps,
    double step_size,
    double goal_bias,
    double rewire_factor,
    double solve_time_seconds,
    double goal_tolerance,
    double validity_resolution
) :
    JointSpacePlanner(model, ik_handle, left_arm, step_size, goal_bias,
                      rewire_factor, solve_time_seconds, goal_tolerance,
                      validity_resolution)
{
    intermediate_pose_offset_ = Eigen::Matrix4d::Identity();
    intermediate_pose_offset_(0, 3) = -approach_offset;

    // Final-leg cartesian poses in goal-relative frame: intermediate -> goal.
    // At t=0 we're at intermediate (offset back from goal); at t=1 we're at goal.
    final_leg_ = linearInterpolate(intermediate_pose_offset_,
                                   Eigen::Matrix4d::Identity(),
                                   final_leg_steps);
}

VisualServoPlanner::~VisualServoPlanner() {}

std::vector<Eigen::Matrix4d> VisualServoPlanner::linearInterpolate(
    const Eigen::Matrix4d& T_from,
    const Eigen::Matrix4d& T_to,
    int steps
) {
    std::vector<Eigen::Matrix4d> out(steps);
    Eigen::Vector3d p_from = T_from.block<3,1>(0,3);
    Eigen::Vector3d p_to   = T_to.block<3,1>(0,3);
    Eigen::Quaterniond q_from(T_from.block<3,3>(0,0));
    Eigen::Quaterniond q_to(T_to.block<3,3>(0,0));

    for (int k = 0; k < steps; ++k) {
        double t = (steps <= 1) ? 0.0 : static_cast<double>(k) / (steps - 1);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = q_from.slerp(t, q_to).toRotationMatrix();
        T.block<3,1>(0,3) = p_from + t * (p_to - p_from);
        out[k] = T;
    }
    return out;
}

std::pair<
std::vector<Eigen::VectorXd>,
std::vector<Eigen::MatrixXd>>
VisualServoPlanner::planPath(
    const Eigen::MatrixXd* goal_pose,
    const Eigen::MatrixXd* start_pose,
    const Eigen::VectorXd* start_vel,
    const Eigen::VectorXd* goal_vel,
    const Eigen::VectorXd* start_acc,
    const Eigen::VectorXd* goal_acc
) {
    if (goal_pose == nullptr) {
        throw std::invalid_argument("VisualServoPlanner::planPath: goal_pose is null");
    }
    if (start_pose == nullptr) {
        throw std::invalid_argument("VisualServoPlanner::planPath: start_pose is null");
    }

    double pos_dist = (goal_pose->block<3,1>(0, 3) - start_pose->block<3,1>(0, 3)).norm();
    bool is_close = pos_dist < 0.05;

    if (is_close) {
        return JointSpacePlanner::planPath(
            goal_pose, start_pose, start_vel, goal_vel, start_acc, goal_acc);
    }

    Eigen::Matrix4d T_intermediate = (*goal_pose) * intermediate_pose_offset_;
    Eigen::MatrixXd intermediate_mat = T_intermediate;

    auto [joints, carts] = JointSpacePlanner::planPath(
        &intermediate_mat, start_pose,
        start_vel, goal_vel, start_acc, goal_acc);
    if (joints.empty()) return {};

    for (const auto& rel : final_leg_) {
        Eigen::Matrix4d T_abs = (*goal_pose) * rel;
        Eigen::VectorXd q = poseToJointConfig(T_abs);
        if (q.size() == 0) {
            std::cout << "[VisualServoPlanner::planPath] final-leg IK failed; truncating" << std::endl;
            break;
        }
        carts.push_back(configToPose(q));
        joints.push_back(std::move(q));
    }
    return {joints, carts};
}

std::pair<
std::vector<Eigen::VectorXd>,
std::vector<Eigen::MatrixXd>>
VisualServoPlanner::planTrajectory(
    const Eigen::MatrixXd* goal_pose,
    const Eigen::MatrixXd* start_pose,
    const Eigen::VectorXd* start_vel,
    const Eigen::VectorXd* goal_vel,
    const Eigen::VectorXd* start_acc,
    const Eigen::VectorXd* goal_acc,
    double time_step,
    const double MAX_LIN_VEL,
    const double MAX_ANG_VEL,
    const double MAX_LIN_ACC,
    const double MAX_ANG_ACC
) {
    if (goal_pose == nullptr) {
        throw std::invalid_argument("VisualServoPlanner::planTrajectory: goal_pose is null");
    }
    if (start_pose == nullptr) {
        throw std::invalid_argument("VisualServoPlanner::planTrajectory: start_pose is null");
    }

    double pos_dist = (goal_pose->block<3,1>(0, 3) - start_pose->block<3,1>(0, 3)).norm();
    bool is_close = pos_dist < 0.05;

    if (is_close) {
        return JointSpacePlanner::planTrajectory(
            goal_pose, start_pose,
            start_vel, goal_vel, start_acc, goal_acc,
            time_step, MAX_LIN_VEL, MAX_ANG_VEL, MAX_LIN_ACC, MAX_ANG_ACC);
    }

    Eigen::Matrix4d T_intermediate = (*goal_pose) * intermediate_pose_offset_;
    Eigen::MatrixXd intermediate_mat = T_intermediate;

    // Phase 1: time-scaled RRT* path to intermediate. Call base planPath
    // explicitly (non-virtual) so we don't recurse into our own override.
    JointSpacePlanner::planPath(
        &intermediate_mat, start_pose,
        start_vel, goal_vel, start_acc, goal_acc);
    if (waypoints_.empty()) return {};

    double phase1_dist = (T_intermediate.block<3,1>(0,3) - start_pose->block<3,1>(0,3)).norm();
    double vel_cap = std::max(MAX_LIN_VEL, 1e-6);
    int steps = std::max(2,
        static_cast<int>(std::ceil((phase1_dist / vel_cap) / std::max(time_step, 1e-6))));

    std::vector<Eigen::VectorXd> joints = resamplePath(waypoints_, steps);
    std::vector<Eigen::MatrixXd> carts = configsToPoses(joints);

    // Phase 2: final-leg cartesian linear approach
    for (const auto& rel : final_leg_) {
        Eigen::Matrix4d T_abs = (*goal_pose) * rel;
        Eigen::VectorXd q = poseToJointConfig(T_abs);
        if (q.size() == 0) {
            std::cout << "[VisualServoPlanner::planTrajectory] final-leg IK failed; truncating" << std::endl;
            break;
        }
        carts.push_back(configToPose(q));
        joints.push_back(std::move(q));
    }
    return {joints, carts};
}

} // namespace HumanoidPilot
