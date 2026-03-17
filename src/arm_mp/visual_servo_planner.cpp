#include "arm_mp/visual_servo_planner.h"
#include <iostream>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>

namespace ArmPilot{

VisualServoPlanner::VisualServoPlanner(double offset, int order) : 
    PolynomialTrajectoryGenerator(order)
{

    intermediate_pose_offset_ = Eigen::Matrix4d::Identity(4,4);
    intermediate_pose_offset_ <<
        1, 0, 0, -offset,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    goal_p_unscaled_ = vee(intermediate_pose_offset_.log());

    start_p_unscaled_ = Eigen::VectorXd::Zero(goal_p_unscaled_.size());
    start_vel_unscaled_ = Eigen::VectorXd::Zero(goal_p_unscaled_.size());
    start_acc_unscaled_ = Eigen::VectorXd::Zero(goal_p_unscaled_.size());
    goal_vel_unscaled_ = Eigen::VectorXd::Zero(goal_p_unscaled_.size());
    goal_acc_unscaled_ = Eigen::VectorXd::Zero(goal_p_unscaled_.size());

    construct_line_(20);

    for (auto twist: line_){
        final_leg_line_.push_back(hat(twist).exp());
    }

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


    double pos_dist = (goal_pose->block<3,0>(0, 3) - start_pose_.block<3,0>(0, 3)).norm();
    bool is_close = pos_dist < 0.1;

    // Building the first part of the path
    if (is_close){
        goal_pose_ = *goal_pose;
    } else {
        goal_pose_ = start_pose_.lu().solve(*goal_pose*intermediate_pose_offset_);
    }
    
    goal_p_unscaled_ = goal_pose_.log();

    construct_line_(steps);

    std::vector<Eigen::MatrixXd> path;

    for (auto twist: line_){
        path.push_back(start_pose_*(hat(twist).exp()));
    }

    if (is_close){

        // Building the second part of the path
        for (auto pose: final_leg_line_){
            path.push_back(goal_pose_*pose);
        }
    }

    last_pose_ = start_pose_;

    return path;

}

} // namespace ArmPilot