#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Eigen/Dense>
#include <vector>

namespace ArmPilot{

class PolynomialTrajectoryGenerator {
    public:
        PolynomialTrajectoryGenerator(int order = 5);
        virtual ~PolynomialTrajectoryGenerator();

        /**
         * @brief Plan end-effector motion trajectory
         * @param start_pose Starting pose of the end-effector (4x4 homogeneous transformation)
         * @param goal_pose Goal pose of the end-effector (4x4 homogeneous transformation)
         * @param duration Duration of the motion in seconds
         * @param time_step Time step for the trajectory points
         * @return Vector of poses representing the trajectory
         */
        std::vector<Eigen::MatrixXd> planTrajectory(
            const Eigen::MatrixXd* goal_pose,
            const Eigen::MatrixXd* start_pose = nullptr,
            double duration = 1,
            double time_step = 0.05,
            const Eigen::VectorXd* start_vel = nullptr,
            const Eigen::VectorXd* goal_vel = nullptr,
            const Eigen::VectorXd* start_acc = nullptr,
            const Eigen::VectorXd* goal_acc = nullptr,
            const double MAX_LIN_VEL = 0.05,
            const double MAX_ANG_VEL = 4.0,
            const double MAX_LIN_ACC = 0.05,
            const double MAX_ANG_ACC = 4.0
        );

        std::vector<Eigen::MatrixXd> planPath(
            const Eigen::MatrixXd* goal_pose,
            const Eigen::MatrixXd* start_pose = nullptr,
            const Eigen::VectorXd* start_vel = nullptr,
            const Eigen::VectorXd* goal_vel = nullptr,
            const Eigen::VectorXd* start_acc = nullptr,
            const Eigen::VectorXd* goal_acc = nullptr,
            int steps = -1
        );

    protected:
        void construct_line_(int steps);

        Eigen::MatrixXd generate_pow_vector_(const Eigen::VectorXd& s, int pow);
        Eigen::VectorXd generate_pow_vector_(double s, int pow);
        Eigen::MatrixXd diff_(int pol_order, int diff_order = 1);
        std::vector<Eigen::VectorXd> evaluate_polynomial(
                    const Eigen::VectorXd& t,
                    const Eigen::MatrixXd& coeff
        );
        Eigen::VectorXd vee(const Eigen::MatrixXd m);
        Eigen::MatrixXd hat(const Eigen::VectorXd v);

        int order_;
        
        Eigen::MatrixXd last_pose_;
        Eigen::MatrixXd start_pose_;
        Eigen::MatrixXd rel_pose_;

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> constraints_;

        Eigen::MatrixXd constraint_coeff_scaling;  // shorthand ccs
        Eigen::MatrixXd ccs_right_inv;

        Eigen::VectorXd start_p_unscaled_;
        Eigen::VectorXd goal_p_unscaled_;
        Eigen::VectorXd start_vel_unscaled_;
        Eigen::VectorXd goal_vel_unscaled_;
        Eigen::VectorXd start_acc_unscaled_;
        Eigen::VectorXd goal_acc_unscaled_;

        std::vector<Eigen::VectorXd> line_;

};



}

#endif // MOTION_PLANNER_H