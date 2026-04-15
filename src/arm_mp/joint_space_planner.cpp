#include "arm_mp/joint_space_planner.h"
#include "base/g1.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ArmPilot {

JointSpacePlanner::JointSpacePlanner(
    pinocchio::Model& model,
    pinocchio::GeometryModel& geom_model,
    HumanoidIK* ik_handle,
    bool left_arm,
    double joint_limit_safety_offset,
    double step_size,
    double goal_bias,
    double rewire_radius,
    int max_iterations,
    double goal_tolerance,
    double approach_offset,
    int final_leg_steps
) :
    model_(model),
    geom_model_(geom_model),
    ik_handle_(ik_handle),
    left_arm_(left_arm),
    nq_(model.nq),
    safety_offset_(joint_limit_safety_offset),
    step_size_(step_size),
    goal_bias_(goal_bias),
    rewire_radius_(rewire_radius),
    max_iterations_(max_iterations),
    goal_tolerance_(goal_tolerance),
    approach_offset_(approach_offset),
    final_leg_steps_(final_leg_steps),
    rng_(std::random_device{}())
{
    intermediate_pose_offset_ = Eigen::Matrix4d::Identity();
    intermediate_pose_offset_(0, 3) = -approach_offset_;
    q_lower_ = model_.lowerPositionLimit.array() + safety_offset_;
    q_upper_ = model_.upperPositionLimit.array() - safety_offset_;

    for (int i = 0; i < nq_; ++i) {
        if (q_lower_(i) > q_upper_(i)) {
            double mid = 0.5 * (model_.lowerPositionLimit(i) + model_.upperPositionLimit(i));
            q_lower_(i) = mid;
            q_upper_(i) = mid;
        }
    }
}

JointSpacePlanner::~JointSpacePlanner() {}

void JointSpacePlanner::setSeed(unsigned int seed) {
    rng_.seed(seed);
}

Eigen::VectorXd JointSpacePlanner::poseToJointConfig(const Eigen::MatrixXd& pose) {
    if (pose.rows() != 4 || pose.cols() != 4) {
        throw std::invalid_argument("JointSpacePlanner: pose must be 4x4 SE(3)");
    }
    if (ik_handle_ == nullptr) {
        throw std::runtime_error("JointSpacePlanner: ik_handle is null");
    }

    Eigen::Matrix4d T = pose;
    JointState js = ik_handle_->solve_ik(T, left_arm_);

    auto [q, v, e] = jointstate_to_vectors(js, model_);
    (void)v; (void)e;
    return q;
}

Eigen::VectorXd JointSpacePlanner::sampleRandom() {
    Eigen::VectorXd q(nq_);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int i = 0; i < nq_; ++i) {
        q(i) = q_lower_(i) + u(rng_) * (q_upper_(i) - q_lower_(i));
    }
    return q;
}

int JointSpacePlanner::nearest(const std::vector<Node>& tree, const Eigen::VectorXd& q) {
    int best = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < tree.size(); ++i) {
        double d = (tree[i].q - q).norm();
        if (d < best_d) {
            best_d = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

std::vector<int> JointSpacePlanner::near(
    const std::vector<Node>& tree, const Eigen::VectorXd& q, double radius
) {
    std::vector<int> out;
    for (size_t i = 0; i < tree.size(); ++i) {
        if ((tree[i].q - q).norm() <= radius) {
            out.push_back(static_cast<int>(i));
        }
    }
    return out;
}

Eigen::VectorXd JointSpacePlanner::steer(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to, double step
) {
    Eigen::VectorXd delta = to - from;
    double d = delta.norm();
    Eigen::VectorXd q_new = (d <= step) ? to : (from + (step / d) * delta);
    q_new = q_new.cwiseMax(q_lower_).cwiseMin(q_upper_);
    return q_new;
}

std::vector<Eigen::VectorXd> JointSpacePlanner::reconstructPath(
    const std::vector<Node>& tree, int goal_idx
) {
    std::vector<Eigen::VectorXd> path;
    int idx = goal_idx;
    while (idx != -1) {
        path.push_back(tree[idx].q);
        idx = tree[idx].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Eigen::VectorXd> JointSpacePlanner::runRRTStar(
    const Eigen::VectorXd& start_q,
    const Eigen::VectorXd& goal_q,
    double step
) {
    std::vector<Node> tree;
    tree.push_back({start_q, -1, 0.0});

    std::uniform_real_distribution<double> u01(0.0, 1.0);

    int best_goal_idx = -1;
    double best_goal_cost = std::numeric_limits<double>::infinity();

    for (int it = 0; it < max_iterations_; ++it) {
        Eigen::VectorXd q_rand = (u01(rng_) < goal_bias_) ? goal_q : sampleRandom();

        int nearest_idx = nearest(tree, q_rand);
        Eigen::VectorXd q_new = steer(tree[nearest_idx].q, q_rand, step);

        std::vector<int> neighbors = near(tree, q_new, rewire_radius_);

        int best_parent = nearest_idx;
        double best_cost = tree[nearest_idx].cost + (tree[nearest_idx].q - q_new).norm();
        for (int ni : neighbors) {
            double c = tree[ni].cost + (tree[ni].q - q_new).norm();
            if (c < best_cost) {
                best_cost = c;
                best_parent = ni;
            }
        }

        int new_idx = static_cast<int>(tree.size());
        tree.push_back({q_new, best_parent, best_cost});

        for (int ni : neighbors) {
            double c = best_cost + (q_new - tree[ni].q).norm();
            if (c < tree[ni].cost) {
                tree[ni].parent = new_idx;
                tree[ni].cost = c;
            }
        }

        double dist_to_goal = (q_new - goal_q).norm();
        if (dist_to_goal <= goal_tolerance_) {
            double total = best_cost + dist_to_goal;
            if (total < best_goal_cost) {
                int goal_idx = static_cast<int>(tree.size());
                tree.push_back({goal_q, new_idx, total});
                best_goal_cost = total;
                best_goal_idx = goal_idx;
            }
        }
    }

    if (best_goal_idx == -1) {
        return {};
    }

    return reconstructPath(tree, best_goal_idx);
}

Eigen::Matrix4d JointSpacePlanner::fkPose(const Eigen::VectorXd& q) {
    pinocchio::Data data(model_);
    pinocchio::framesForwardKinematics(model_, data, q);

    const std::string frame_name = left_arm_ ? "L_ee" : "R_ee";
    if (!model_.existFrame(frame_name)) {
        return Eigen::Matrix4d::Identity();
    }
    pinocchio::FrameIndex fid = model_.getFrameId(frame_name);
    return data.oMf[fid].toHomogeneousMatrix();
}

std::vector<Eigen::MatrixXd> JointSpacePlanner::densify(
    const std::vector<Eigen::VectorXd>& path, int steps
) {
    std::vector<Eigen::MatrixXd> out;
    if (path.empty()) return out;

    std::vector<Eigen::VectorXd> dense_q;

    if (steps <= 0 || static_cast<int>(path.size()) >= steps) {
        dense_q = path;
    } else {
        std::vector<double> seg_len(path.size() - 1);
        double total = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            seg_len[i] = (path[i + 1] - path[i]).norm();
            total += seg_len[i];
        }

        if (total <= 0.0) {
            dense_q.push_back(path.front());
        } else {
            dense_q.reserve(steps);
            for (int k = 0; k < steps; ++k) {
                double s = (static_cast<double>(k) / (steps - 1)) * total;
                double acc = 0.0;
                size_t i = 0;
                for (; i + 1 < path.size(); ++i) {
                    if (acc + seg_len[i] >= s) break;
                    acc += seg_len[i];
                }
                if (i + 1 >= path.size()) {
                    dense_q.push_back(path.back());
                    continue;
                }
                double t = (seg_len[i] > 0.0) ? (s - acc) / seg_len[i] : 0.0;
                dense_q.push_back(path[i] + t * (path[i + 1] - path[i]));
            }
        }
    }

    out.reserve(dense_q.size());
    for (const auto& q : dense_q) {
        out.push_back(fkPose(q));
    }
    return out;
}

JointSpacePlanner::TwoPhaseLegs JointSpacePlanner::planTwoPhaseLegs(
    const Eigen::MatrixXd& goal_pose,
    const Eigen::MatrixXd* start_pose
) {
    TwoPhaseLegs out;

    Eigen::Matrix4d T_goal = goal_pose;
    out.T_intermediate = T_goal * intermediate_pose_offset_;

    Eigen::VectorXd goal_q = poseToJointConfig(T_goal);
    goal_q = goal_q.cwiseMax(q_lower_).cwiseMin(q_upper_);

    Eigen::VectorXd start_q;
    if (start_pose != nullptr) {
        out.T_start = *start_pose;
        start_q = poseToJointConfig(out.T_start);
    } else {
        out.T_start = Eigen::Matrix4d::Identity();
        start_q = Eigen::VectorXd::Zero(nq_);
    }
    start_q = start_q.cwiseMax(q_lower_).cwiseMin(q_upper_);

    double translational_dist =
        (T_goal.block<3,1>(0,3) - out.T_start.block<3,1>(0,3)).norm();
    out.is_close = translational_dist < 2.0 * approach_offset_;

    if (out.is_close) {
        out.leg1 = runRRTStar(start_q, goal_q, step_size_);
    } else {
        Eigen::VectorXd intermediate_q = poseToJointConfig(out.T_intermediate);
        intermediate_q = intermediate_q.cwiseMax(q_lower_).cwiseMin(q_upper_);

        out.leg1 = runRRTStar(start_q, intermediate_q, step_size_);

        int n2 = std::max(2, final_leg_steps_);
        out.leg2.reserve(n2);
        for (int k = 1; k <= n2; ++k) {
            double t = static_cast<double>(k) / n2;
            Eigen::Matrix4d T = interpolate_poses(out.T_intermediate, T_goal, t);
            out.leg2.push_back(T);
        }
    }

    return out;
}

std::vector<Eigen::VectorXd> JointSpacePlanner::planTwoPhase(
    const Eigen::MatrixXd& goal_pose,
    const Eigen::MatrixXd* start_pose
) {
    TwoPhaseLegs legs = planTwoPhaseLegs(goal_pose, start_pose);
    std::vector<Eigen::VectorXd> out = std::move(legs.leg1);
    for (auto& q : legs.leg2) out.push_back(std::move(q));
    return out;
}

std::vector<Eigen::VectorXd> JointSpacePlanner::resampleLinear(
    const std::vector<Eigen::VectorXd>& path, int steps
) {
    std::vector<Eigen::VectorXd> out;
    if (path.empty()) return out;
    if (steps <= 1 || path.size() == 1) {
        out = path;
        return out;
    }

    std::vector<double> seg_len(path.size() - 1);
    double total = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        seg_len[i] = (path[i + 1] - path[i]).norm();
        total += seg_len[i];
    }
    if (total <= 0.0) {
        out.push_back(path.front());
        return out;
    }

    out.reserve(steps);
    for (int k = 0; k < steps; ++k) {
        double s = (static_cast<double>(k) / (steps - 1)) * total;
        double acc = 0.0;
        size_t i = 0;
        for (; i + 1 < path.size(); ++i) {
            if (acc + seg_len[i] >= s) break;
            acc += seg_len[i];
        }
        if (i + 1 >= path.size()) {
            out.push_back(path.back());
            continue;
        }
        double t = (seg_len[i] > 0.0) ? (s - acc) / seg_len[i] : 0.0;
        out.push_back(path[i] + t * (path[i + 1] - path[i]));
    }
    return out;
}

std::vector<Eigen::MatrixXd> JointSpacePlanner::planPath(
    const Eigen::MatrixXd* goal_pose,
    const Eigen::MatrixXd* start_pose,
    const Eigen::VectorXd* /*start_vel*/,
    const Eigen::VectorXd* /*goal_vel*/,
    const Eigen::VectorXd* /*start_acc*/,
    const Eigen::VectorXd* /*goal_acc*/,
    int steps
) {
    if (goal_pose == nullptr) {
        throw std::invalid_argument("JointSpacePlanner::planPath: goal_pose is null");
    }

    TwoPhaseLegs legs = planTwoPhaseLegs(*goal_pose, start_pose);
    if (legs.leg1.empty()) return {};

    std::vector<Eigen::VectorXd> leg1 = (steps > 0)
        ? resampleLinear(legs.leg1, steps)
        : legs.leg1;

    std::vector<Eigen::MatrixXd> out;
    out.reserve(leg1.size() + legs.leg2.size());
    for (const auto& q : leg1) out.push_back(fkPose(q));
    for (const auto& T : legs.leg2) out.push_back(T);
    return out;
}

std::vector<Eigen::MatrixXd> JointSpacePlanner::planTrajectory(
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

    (void)start_vel; (void)goal_vel; (void)start_acc; (void)goal_acc;

    TwoPhaseLegs legs = planTwoPhaseLegs(*goal_pose, start_pose);
    if (legs.leg1.empty()) return {};

    // VSP-style step count for Phase 1: based on task-space translational
    // distance of the Phase 1 segment (start -> intermediate) at MAX_LIN_VEL.
    Eigen::Vector3d p_from = legs.T_start.block<3,1>(0,3);
    Eigen::Vector3d p_to   = legs.is_close
        ? Eigen::Vector3d(goal_pose->block<3,1>(0,3))
        : Eigen::Vector3d(legs.T_intermediate.block<3,1>(0,3));
    double task_dist = (p_to - p_from).norm();

    double vel_cap = std::max(MAX_LIN_VEL, 1e-6);
    double duration = task_dist / vel_cap;
    int leg1_steps = std::max(2,
        static_cast<int>(std::ceil(duration / std::max(time_step, 1e-6))));

    std::vector<Eigen::VectorXd> leg1 = resampleLinear(legs.leg1, leg1_steps);

    std::vector<Eigen::MatrixXd> out;
    out.reserve(leg1.size() + legs.leg2.size());
    for (const auto& q : leg1) out.push_back(fkPose(q));
    for (const auto& T : legs.leg2) out.push_back(T);
    return out;
}

Eigen::MatrixXd JointSpacePlanner::interpolate_poses(const Eigen::MatrixXd& T1, const Eigen::MatrixXd& T2, double t) {
    Eigen::Vector3d pos1 = T1.block<3,1>(0,3);
    Eigen::Vector3d pos2 = T2.block<3,1>(0,3);
    Eigen::Vector3d pos_interp = pos1 + t * (pos2 - pos1);

    // 2. Interpolate Rotation (SLERP)
    // We cast the 3x3 block to a Quaternion
    Eigen::Quaterniond q1(T1.block<3,3>(0,0));
    Eigen::Quaterniond q2(T2.block<3,3>(0,0));
    
    // Slerp handles the spherical path between orientations
    Eigen::Quaterniond rot_interp = q1.slerp(t, q2);

    // 3. Reconstruct as MatrixXd
    Eigen::MatrixXd T_out = Eigen::MatrixXd::Identity(4, 4);
    T_out.block<3,3>(0,0) = rot_interp.toRotationMatrix();
    T_out.block<3,1>(0,3) = pos_interp;
    
    return T_out;
}

} // namespace ArmPilot
