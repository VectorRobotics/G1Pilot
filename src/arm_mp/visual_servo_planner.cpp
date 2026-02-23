#include "arm_mp/visual_servo_planner.h"
#include <iostream>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>

namespace ArmPilot{

VisualServoPlanner::VisualServoPlanner(double offset, int order) : 
    PolynomialTrajectoryGenerator(order)
{
    last_pose_ = Eigen::MatrixXd::Identity(4, 4);

    constraint_coeff_scaling.resize(order_+1,7);
    constraint_coeff_scaling <<
        generate_pow_vector_(0.0, order_),
        generate_pow_vector_(1.0, order_),
        generate_pow_vector_(0.9, order_),
        diff_(order_) * generate_pow_vector_(0.0, order_ - 1),
        diff_(order_) * generate_pow_vector_(1.0, order_ - 1),
        diff_(order_, 2) * generate_pow_vector_(0.0, order_ - 2),
        diff_(order_, 2) * generate_pow_vector_(1.0, order_ - 2);

    if (6>order_+1) {
        Eigen::MatrixXd ccsccsT = constraint_coeff_scaling*constraint_coeff_scaling.transpose();
        ccs_right_inv = constraint_coeff_scaling.transpose().llt().solve(ccsccsT);
    } else {
        Eigen::MatrixXd ccsTccs = constraint_coeff_scaling.transpose()*constraint_coeff_scaling;
        ccs_right_inv = ccsTccs.llt().solve(constraint_coeff_scaling.transpose());
    }

    intermediate_pose_offset_ = Eigen::Matrix4d::Identity(4,4);
    intermediate_pose_offset_ <<
        1, 0, 0, -offset,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    std::cout<<"Trajecotry Generator Initialized"<<std::endl;

}

VisualServoPlanner::~VisualServoPlanner() {}


std::vector<Eigen::MatrixXd> 
VisualServoPlanner::planPath(
    const Eigen::MatrixXd* goal_pose,
    const Eigen::MatrixXd* start_pose,
    const Eigen::VectorXd* start_vel,
    const Eigen::VectorXd* goal_vel,
    const Eigen::VectorXd* start_acc,
    const Eigen::VectorXd* goal_acc,
    int steps
) {
    assert(2*(goal_pose->cols()-1)==start_vel->rows());

    int twist_size = 2*(goal_pose->rows()-1);

    rel_pose_ = start_pose->lu().solve(*goal_pose).log();
    intermediate_pose_ = start_pose->lu().solve(*goal_pose*intermediate_pose_offset_).log();

    goal_p_unscaled_ = vee(rel_pose_);
    intermediate_p_unscaled_ = vee(intermediate_pose_);

    start_p_unscaled_ = Eigen::VectorXd::Zero(goal_p_unscaled_.size());
    
    if (start_pose!=nullptr) 
        start_pose_ = *start_pose; 
    else 
        start_pose_ = Eigen::MatrixXd::Identity(goal_pose->rows(), goal_pose->cols());

    if (start_vel!=nullptr) 
        start_vel_unscaled_ = *start_vel; 
    else 
        start_vel_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    if (start_acc!=nullptr) 
        start_acc_unscaled_ = *start_acc; 
    else 
        start_acc_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    if (goal_vel!=nullptr) 
        goal_vel_unscaled_ = *goal_vel; 
    else 
        goal_vel_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    if (goal_acc!=nullptr) 
        goal_acc_unscaled_ = *goal_acc; 
    else 
        goal_acc_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    construct_line_(steps);

    std::vector<Eigen::MatrixXd> path;

    for (auto twist: line_){
        path.push_back(start_pose_*(hat(twist).exp()));
    }

    last_pose_ = start_pose_;

    return path;

}

void VisualServoPlanner::construct_line_(int steps)
{

    double dist_to_int = std::sqrt(
        goal_p_unscaled_.head<3>().squaredNorm() +
        goal_p_unscaled_.tail<3>().squaredNorm()*0.09
    );
    
    if (dist_to_int < 0.06){
        constraint_coeff_scaling.resize(order_+1,6);
        constraint_coeff_scaling <<
            generate_pow_vector_(0.0, order_),
            generate_pow_vector_(1.0, order_),
            diff_(order_) * generate_pow_vector_(0.0, order_ - 1),
            diff_(order_) * generate_pow_vector_(1.0, order_ - 1),
            diff_(order_, 2) * generate_pow_vector_(0.0, order_ - 2),
            diff_(order_, 2) * generate_pow_vector_(1.0, order_ - 2);
    
        if (6>order_+1) {
            Eigen::MatrixXd ccsccsT = constraint_coeff_scaling*constraint_coeff_scaling.transpose();
            ccs_right_inv = constraint_coeff_scaling.transpose().llt().solve(ccsccsT);
        } else {
            Eigen::MatrixXd ccsTccs = constraint_coeff_scaling.transpose()*constraint_coeff_scaling;
            ccs_right_inv = ccsTccs.llt().solve(constraint_coeff_scaling.transpose());
        }
    
        PolynomialTrajectoryGenerator::construct_line_(steps);
        return;
    }
    
    // coeff * A = b
    Eigen::MatrixXd b(goal_p_unscaled_.size(), 7);
    Eigen::MatrixXd coeff(goal_p_unscaled_.size(), order_+1);
    
    b <<
        start_p_unscaled_,
        goal_p_unscaled_,
        intermediate_p_unscaled_,
        start_vel_unscaled_,
        goal_vel_unscaled_,
        start_acc_unscaled_,
        goal_acc_unscaled_;
    
    coeff = b*ccs_right_inv;
    
    Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(steps, 0, 1);
    
    line_ = evaluate_polynomial(t, coeff);
}

} // namespace ArmPilot