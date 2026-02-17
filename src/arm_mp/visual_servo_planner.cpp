#include "arm_mp/visual_servo_planner.h"
#include <iostream>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace ArmPilot{

VisualServoPlanner::VisualServoPlanner(double offset, int order) : 
    PolynomialTrajectoryGenerator(order)
{

    constraint_coeff_scaling.resize(order_+1,7);
    constraint_coeff_scaling <<
        generate_pow_vector_(0.0, order_),
        generate_pow_vector_(1.0, order_),
        generate_pow_vector_(0.7, order_),
        diff_(order_) * generate_pow_vector_(0.0, order_ - 1),
        diff_(order_) * generate_pow_vector_(1.0, order_ - 1),
        diff_(order_, 2) * generate_pow_vector_(0.0, order_ - 2),
        diff_(order_, 2) * generate_pow_vector_(1.0, order_ - 2);

    if (7>order_+1) {
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

    // Compute start^{-1} * goal using SE(3) structure (avoids LU numerical noise)
    Eigen::Matrix3d R_s = start_pose->block<3,3>(0,0);
    Eigen::Vector3d t_s = start_pose->block<3,1>(0,3);
    Eigen::Matrix3d R_g = goal_pose->block<3,3>(0,0);
    Eigen::Vector3d t_g = goal_pose->block<3,1>(0,3);
    pinocchio::SE3 rel_se3(R_s.transpose() * R_g, R_s.transpose() * (t_g - t_s));
    goal_p_unscaled_ = pinocchio::log6(rel_se3).toVector();

    Eigen::MatrixXd goal_inter = (*goal_pose) * intermediate_pose_offset_;
    Eigen::Matrix3d R_i = goal_inter.block<3,3>(0,0);
    Eigen::Vector3d t_i = goal_inter.block<3,1>(0,3);
    pinocchio::SE3 inter_se3(R_s.transpose() * R_i, R_s.transpose() * (t_i - t_s));
    intermediate_p_unscaled_ = pinocchio::log6(inter_se3).toVector();

    // std::cout << "Constructing Path: \n" << std::endl;
    // std::cout << "start: \n" << *start_pose << std::endl;
    // std::cout << "end: \n" << *goal_pose << std::endl;
    // std::cout << "rel: \n" << rel_pose_ << std::endl;
    // std::cout << "goal t \n" << goal_p_unscaled_ <<std::endl; 
    // std::cout << "int: \n" << start_pose->lu().solve(*goal_pose*intermediate_pose_offset_) <<std::endl; 
    // std::cout << "int unscaled: \n" << intermediate_p_unscaled_ <<std::endl; 

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
    // std::cout << "Constructing Line" << std::endl;
    // std::cout << "start" << start_p_unscaled_ << std::endl;
    // std::cout << "end" << goal_p_unscaled_ << std::endl;

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

    // std::cout << std::fixed << std::setprecision(2);
    // std::cout << "coeff: \n" << coeff << std::endl;

    Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(steps, 0, 1);

    line_ = evaluate_polynomial(t, coeff);

}

} // namespace ArmPilot