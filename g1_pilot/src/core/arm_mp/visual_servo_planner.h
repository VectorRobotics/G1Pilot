#ifndef VISUAL_SERVO_PLANNER_H
#define VISUAL_SERVO_PLANNER_H

#include "core/arm_mp/joint_space_planner.h"

namespace HumanoidPilot{

class VisualServoPlanner : public JointSpacePlanner {
    public:
        VisualServoPlanner(
            pinocchio::Model& model,
            std::unique_ptr<HumanoidIK> ik_handle,
            bool left_arm = false,
            double approach_offset = 0.05,
            int final_leg_steps = 20,
            double step_size = 0.05,
            double goal_bias = 0.1,
            double rewire_factor = 1.1,
            double solve_time_seconds = 1.0,
            double goal_tolerance = 0.01,
            double validity_resolution = 0.005
        );
        virtual ~VisualServoPlanner();

        std::pair<
        std::vector<Eigen::VectorXd>,
        std::vector<Eigen::MatrixXd>>
        planPath(
            const Eigen::MatrixXd* goal_pose,
            const Eigen::MatrixXd* start_pose = nullptr,
            const Eigen::VectorXd* start_vel = nullptr,
            const Eigen::VectorXd* goal_vel = nullptr,
            const Eigen::VectorXd* start_acc = nullptr,
            const Eigen::VectorXd* goal_acc = nullptr
        ) override;

        std::pair<
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
        ) override;

    protected:
        std::vector<Eigen::Matrix4d> linearInterpolate(
            const Eigen::Matrix4d& T_from,
            const Eigen::Matrix4d& T_to,
            int steps
        );

        Eigen::Matrix4d intermediate_pose_offset_;
        std::vector<Eigen::Matrix4d> final_leg_;
};

} // namespace HumanoidPilot

#endif // VISUAL_SERVO_PLANNER_H
