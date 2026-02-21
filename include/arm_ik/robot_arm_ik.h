#ifndef ROBOT_ARM_IK_H
#define ROBOT_ARM_IK_H

#include "utils.h"
#include "../base/interfaces.h"
#include "weighted_moving_filter.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>


namespace ArmPilot {

class HumanoidIK {
public:
    HumanoidIK(
        pinocchio::Model& model,
        pinocchio::GeometryModel& geom_model
    );

    virtual JointState solve_ik(
        const Eigen::Matrix4d& left_wrist,
        const Eigen::Matrix4d& right_wrist,
        const Eigen::VectorXd* current_lr_arm_motor_q = nullptr,
        const Eigen::VectorXd* current_lr_arm_motor_dq = nullptr,
        const Eigen::VectorXd* EE_efrc_L = nullptr,
        const Eigen::VectorXd* EE_efrc_R = nullptr
    );

protected:
    void setup_optimization();
    void filter_adjacent_collision_pairs();

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel geom_model_;
    pinocchio::GeometryData geom_data_;

    #ifdef USE_CASADI

        casadi::Opti opti_;
        casadi::MX var_q_;
        casadi::MX var_q_last_;
        casadi::MX param_tf_l_;
        casadi::MX param_tf_r_;

    #else // USE_CASADI

        const double eps  = 1e-4;
        const int IT_MAX  = 1000;
        const double DT   = 1e-1;
        const double damp = 1e-6;

        Eigen::VectorXd var_q_;
        Eigen::VectorXd var_q_last_;
        pinocchio::SE3 param_tf_l_;
        pinocchio::SE3 param_tf_r_;
        Eigen::Matrix<double, 12,1> err;

    #endif // USE_CASADI

    pinocchio::FrameIndex L_hand_id_;
    pinocchio::FrameIndex R_hand_id_;

    int nq_;
    int nv_;

    pinocchio::FrameIndex oMcamera;
    pinocchio::FrameIndex oMLidar;

    Eigen::VectorXd init_data_;
    std::unique_ptr<WeightedMovingFilter> smooth_filter_;

    Eigen::VectorXd sol_q;
    Eigen::VectorXd sol_v;
    Eigen::VectorXd sol_t;
    Eigen::VectorXd sol_t_ext_L;
    Eigen::VectorXd sol_t_ext_R;

    pinocchio::Data::Matrix6x J_L;
    pinocchio::Data::Matrix6x J_R;

};

} // ArmPilot namespace
#endif // ROBOT_ARM_IK_H