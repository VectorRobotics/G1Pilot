#ifndef VISUAL_SERVO_PLANNER_H
#define VISUAL_SERVO_PLANNER_H

#include "poly_traj_gen.h"


namespace HumanoidPilot{

class VisualServoPlanner : public PolynomialTrajectoryGenerator {
    public:
        VisualServoPlanner(double z_offset = 0.05, int order = 5);
        virtual ~VisualServoPlanner();

        std::vector<Eigen::MatrixXd> 
        planPath(
            const Eigen::MatrixXd* goal_pose,
            const Eigen::MatrixXd* start_pose,
            const Eigen::VectorXd* start_vel,
            const Eigen::VectorXd* goal_vel,
            const Eigen::VectorXd* start_acc,
            const Eigen::VectorXd* goal_acc,
            int steps
        );

    protected:

        Eigen::MatrixXd intermediate_pose_offset_;
        Eigen::MatrixXd intermediate_pose_;

        Eigen::MatrixXd intermediate_p_unscaled_;

        std::vector<Eigen::MatrixXd> final_leg_line_;
        Eigen::MatrixXd goal_pose_;

};



}

#endif // VISUAL_SERVO_PLANNER_H