#ifndef JOINT_SPACE_PLANNER_H
#define JOINT_SPACE_PLANNER_H

#include "../arm_ik/arm_ik.h"

#include <memory>

namespace ompl {
namespace base {
class StateSpace;
class SpaceInformation;
}
namespace geometric {
class RRTstar;
}
}

namespace HumanoidPilot {

/**
 * @brief OMPL RRT*-based joint-space motion planner.
 *
 * Matches the PolynomialTrajectoryGenerator / VisualServoPlanner interface:
 * planTrajectory(...) and planPath(...) accept 4x4 SE(3) goal/start poses.
 * The provided HumanoidIK handle converts task-space poses into joint-space
 * configurations and provides collision checking; joint limits come from the
 * pinocchio model and are enforced by OMPL bounds and the IK solver.
 */
class JointSpacePlanner {
    public:
        JointSpacePlanner(
            pinocchio::Model& model,
            HumanoidIK* ik_handle,
            bool left_arm = false,
            double step_size = 0.05,
            double goal_bias = 0.1,
            double rewire_factor = 1.1,
            double solve_time_seconds = 1.0,
            double goal_tolerance = 0.01,
            double validity_resolution = 0.005
        );
        virtual ~JointSpacePlanner();

        /**
         * @param goal_pose  4x4 SE(3) goal end-effector pose
         * @param start_pose 4x4 SE(3) start end-effector pose (nullptr -> identity)
         * Velocity/acceleration arguments are accepted for interface parity
         * but ignored (RRT* is kinematic).
         */
        virtual std::pair<
        std::vector<Eigen::VectorXd>,
        std::vector<Eigen::MatrixXd>>
        planTrajectory(
            const Eigen::MatrixXd* goal_pose,
            const Eigen::MatrixXd* start_pose = nullptr,
            const Eigen::VectorXd* start_vel = nullptr,
            const Eigen::VectorXd* goal_vel = nullptr,
            const Eigen::VectorXd* start_acc = nullptr,
            const Eigen::VectorXd* goal_acc = nullptr,
            double time_step = 0.1,
            const double MAX_LIN_VEL = 0.1,
            const double MAX_ANG_VEL = 4.0,
            const double MAX_LIN_ACC = 0.05,
            const double MAX_ANG_ACC = 4.0
        );

        virtual std::pair<
        std::vector<Eigen::VectorXd>,
        std::vector<Eigen::MatrixXd>>
        planPath(
            const Eigen::MatrixXd* goal_pose,
            const Eigen::MatrixXd* start_pose = nullptr,
            const Eigen::VectorXd* start_vel = nullptr,
            const Eigen::VectorXd* goal_vel = nullptr,
            const Eigen::VectorXd* start_acc = nullptr,
            const Eigen::VectorXd* goal_acc = nullptr
        );

        void setLeftArm(bool left_arm) { left_arm_ = left_arm; }

    protected:
        std::vector<Eigen::VectorXd> runRRTStar(
            const Eigen::VectorXd& start_q,
            const Eigen::VectorXd& goal_q
        );

        std::vector<Eigen::VectorXd> resamplePath(
            const std::vector<Eigen::VectorXd>& waypoints, int steps
        );

        Eigen::VectorXd poseToJointConfig(const Eigen::MatrixXd& pose);
        Eigen::Matrix4d configToPose(const Eigen::VectorXd& q);
        std::vector<Eigen::MatrixXd> configsToPoses(
            const std::vector<Eigen::VectorXd>& path
        );

        pinocchio::Model model_;
        pinocchio::Data data_;
        HumanoidIK* ik_handle_;
        bool left_arm_;

        int nq_;
        pinocchio::FrameIndex left_ee_id_;
        pinocchio::FrameIndex right_ee_id_;

        double step_size_;
        double goal_bias_;
        double rewire_factor_;
        double solve_time_seconds_;
        double goal_tolerance_;
        double validity_resolution_;

        std::shared_ptr<ompl::base::StateSpace> ompl_space_;
        std::shared_ptr<ompl::base::SpaceInformation> ompl_si_;
        std::shared_ptr<ompl::geometric::RRTstar> planner_;

        Eigen::VectorXd start_q_;
        Eigen::VectorXd goal_q_;
        std::vector<Eigen::VectorXd> waypoints_;
};

} // namespace HumanoidPilot

#endif // JOINT_SPACE_PLANNER_H
