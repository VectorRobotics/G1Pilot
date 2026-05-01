#ifndef JOINT_SPACE_PLANNER_H
#define JOINT_SPACE_PLANNER_H

#include "../arm_ik/arm_ik.h"

#include <random>

namespace ArmPilot {

/**
 * @brief RRT*-based joint-space motion planner.
 *
 * Matches the PolynomialTrajectoryGenerator / VisualServoPlanner interface:
 * planTrajectory(...) and planPath(...) accept 4x4 SE(3) goal/start poses.
 * The provided HumanoidIK handle converts task-space poses into joint-space
 * configurations, then RRT* plans in joint space with joint limits enforced
 * (with a safety offset).
 *
 * No collision checking is performed.
 */
class JointSpacePlanner {
    public:
        JointSpacePlanner(
            pinocchio::Model& model,
            pinocchio::GeometryModel& geom_model,
            HumanoidIK* ik_handle,
            bool left_arm = false,
            double joint_limit_safety_offset = 0.05,
            double step_size = 0.05,
            double goal_bias = 0.1,
            double rewire_radius = 0.5,
            int max_iterations = 2000,
            double goal_tolerance = 0.01,
            double approach_offset = 0.1,
            int final_leg_steps = 10
        );
        virtual ~JointSpacePlanner();

        /**
         * @param goal_pose  4x4 SE(3) goal end-effector pose
         * @param start_pose 4x4 SE(3) start end-effector pose (nullptr -> identity)
         * Velocity/acceleration arguments are accepted for interface parity
         * but ignored (RRT* is kinematic).
         */
        virtual std::vector<Eigen::MatrixXd> planTrajectory(
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

        virtual std::vector<Eigen::MatrixXd> planPath(
            const Eigen::MatrixXd* goal_pose,
            const Eigen::MatrixXd* start_pose = nullptr,
            const Eigen::VectorXd* start_vel = nullptr,
            const Eigen::VectorXd* goal_vel = nullptr,
            const Eigen::VectorXd* start_acc = nullptr,
            const Eigen::VectorXd* goal_acc = nullptr,
            int steps = -1
        );

        void setSeed(unsigned int seed);
        void setLeftArm(bool left_arm) { left_arm_ = left_arm; }

    protected:
        struct Node {
            Eigen::VectorXd q;
            int parent;
            double cost;
        };

        std::vector<Eigen::VectorXd> runRRTStar(
            const Eigen::VectorXd& start_q,
            const Eigen::VectorXd& goal_q,
            double step
        );

        std::vector<Eigen::VectorXd> planTwoPhase(
            const Eigen::MatrixXd& goal_pose,
            const Eigen::MatrixXd* start_pose
        );

        struct TwoPhaseLegs {
            std::vector<Eigen::VectorXd> leg1;      // start -> intermediate (RRT*)
            std::vector<Eigen::MatrixXd> leg2;      // intermediate -> goal  (straight)
            Eigen::Matrix4d T_start;
            Eigen::Matrix4d T_intermediate;
            bool is_close;
        };

        TwoPhaseLegs planTwoPhaseLegs(
            const Eigen::MatrixXd& goal_pose,
            const Eigen::MatrixXd* start_pose
        );

        std::vector<Eigen::VectorXd> resampleLinear(
            const std::vector<Eigen::VectorXd>& path, int steps
        );

        Eigen::VectorXd poseToJointConfig(const Eigen::MatrixXd& pose);
        Eigen::Matrix4d fkPose(const Eigen::VectorXd& q);

        Eigen::VectorXd sampleRandom();
        int nearest(const std::vector<Node>& tree, const Eigen::VectorXd& q);
        std::vector<int> near(const std::vector<Node>& tree, const Eigen::VectorXd& q, double radius);
        Eigen::VectorXd steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to, double step);
        std::vector<Eigen::VectorXd> reconstructPath(const std::vector<Node>& tree, int goal_idx);

        std::vector<Eigen::MatrixXd> densify(
            const std::vector<Eigen::VectorXd>& path, int steps
        );

        Eigen::MatrixXd interpolate_poses(const Eigen::MatrixXd& T1, const Eigen::MatrixXd& T2, double t);

        pinocchio::Model model_;
        pinocchio::GeometryModel geom_model_;
        HumanoidIK* ik_handle_;
        bool left_arm_;

        int nq_;

        Eigen::VectorXd q_lower_;
        Eigen::VectorXd q_upper_;

        double safety_offset_;
        double step_size_;
        double goal_bias_;
        double rewire_radius_;
        int max_iterations_;
        double goal_tolerance_;

        double approach_offset_;
        int final_leg_steps_;
        Eigen::Matrix4d intermediate_pose_offset_;

        std::mt19937 rng_;
};

} // namespace ArmPilot

#endif // JOINT_SPACE_PLANNER_H
