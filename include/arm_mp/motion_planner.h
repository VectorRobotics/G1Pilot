#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Eigen/Dense>
#include <vector>

namespace MP{

class EE_MotionPlanner {
    public:
        EE_MotionPlanner(int order);
        virtual ~EE_MotionPlanner();

        /**
         * @brief Plan end-effector motion trajectory
         * @param start_pose Starting pose of the end-effector (4x4 homogeneous transformation)
         * @param goal_pose Goal pose of the end-effector (4x4 homogeneous transformation)
         * @param duration Duration of the motion in seconds
         * @param time_step Time step for the trajectory points
         * @return Vector of poses representing the trajectory
         */
        std::vector<Eigen::Matrix4d> planTrajectory(
            const Eigen::Matrix4d* start_pose,
            const Eigen::Matrix4d* goal_pose,
            double duration = 1,
            double time_step = 0.05,
            const Eigen::Vector3d* start_vel = nullptr,
            const Eigen::Vector3d* goal_vel = nullptr,
            const Eigen::Vector3d* start_acc = nullptr,
            const Eigen::Vector3d* goal_acc = nullptr,
            const double MAX_LIN_VEL = 0.05,
            const double MAX_ANG_VEL = 4.0,
            const double MAX_LIN_ACC = 0.05,
            const double MAX_ANG_ACC = 4.0
        );

        std::vector<Eigen::Matrix4d> planPath(
            const Eigen::Matrix4d* start_pose,
            const Eigen::Matrix4d* goal_pose,
            const Eigen::Vector3d* start_vel,
            const Eigen::Vector3d* goal_vel,
            const Eigen::Vector3d* start_acc,
            const Eigen::Vector3d* goal_acc,
            int steps
        );

    // protected:

        std::vector<Eigen::VectorXd> construct_line_(
            Eigen::Vector3d start_pose,
            Eigen::Vector3d goal_pose,
            const Eigen::Vector3d* start_vel,
            const Eigen::Vector3d* goal_vel,
            const Eigen::Vector3d* start_acc,
            const Eigen::Vector3d* goal_acc,
            int steps
        );

        Eigen::MatrixXd generate_pow_vector_(const Eigen::VectorXd& s, int pow);
        Eigen::VectorXd generate_pow_vector_(double s, int pow);
        Eigen::MatrixXd diff_(int pol_order, int diff_order = 1);
        std::vector<Eigen::VectorXd> evaluate_polynomial(
                    const Eigen::VectorXd& t,
                    const Eigen::MatrixXd& coeff
        );

        int order_;
        // int num_constraints_;
        
        Eigen::Matrix4d last_pose_;
        Eigen::Matrix4d star_pose_;
    
        Eigen::Vector3d goal_position_;
        Eigen::Matrix3d goal_orientation_;

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> constraints_;

        

};



}

#endif // MOTION_PLANNER_H