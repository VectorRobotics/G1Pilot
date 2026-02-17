#include "arm_mp/poly_traj_gen.h"
#include <iostream>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace ArmPilot{

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator(int order) : 
    order_(order) 
{
    last_pose_ = Eigen::MatrixXd::Identity(4, 4);

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

    std::cout<<"Trajecotry Generator Initialized"<<std::endl;

}
PolynomialTrajectoryGenerator::~PolynomialTrajectoryGenerator() {}

std::vector<Eigen::MatrixXd> PolynomialTrajectoryGenerator::planTrajectory(
    const Eigen::MatrixXd* goal_pose,
    const Eigen::MatrixXd* start_pose,
    const Eigen::VectorXd* start_vel,
    const Eigen::VectorXd* goal_vel,
    const Eigen::VectorXd* start_acc,
    const Eigen::VectorXd* goal_acc,
    double time_step,
    const double MAX_LIN_VEL,
    const double MAX_ANG_VEL,
    const double MAX_LIN_ACC,
    const double MAX_ANG_ACC
) {
    int steps = 10;
    double duration = steps*time_step;
    int twist_size = 2*(goal_pose->rows()-1);

    if (start_pose!=nullptr) 
        start_pose_ = *start_pose; 
    else 
        start_pose_ = Eigen::MatrixXd::Identity(goal_pose->rows(), goal_pose->cols());

    if (start_vel!=nullptr) 
        start_vel_unscaled_ = *start_vel*duration; 
    else 
        start_vel_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    if (start_acc!=nullptr) 
        start_acc_unscaled_ = *start_acc*duration*duration; 
    else 
        start_acc_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    if (goal_vel!=nullptr) 
        goal_vel_unscaled_ = *goal_vel*duration; 
    else 
        goal_vel_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    if (goal_acc!=nullptr) 
        goal_acc_unscaled_ = *goal_acc*duration*duration; 
    else 
        goal_acc_unscaled_ = Eigen::VectorXd::Zero(twist_size);

    std::vector<Eigen::MatrixXd> path = planPath(
        goal_pose,
        &start_pose_,
        &start_vel_unscaled_,
        &goal_vel_unscaled_,
        &start_acc_unscaled_,
        &goal_acc_unscaled_,
        steps
    );

    return path;

}

std::vector<Eigen::MatrixXd> 
PolynomialTrajectoryGenerator::planPath(
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

    // std::cout << "Constructing Path" << std::endl;
    // std::cout << "start" << *start_pose << std::endl;
    // std::cout << "end" << *goal_pose << std::endl;
    // std::cout << "rel" << rel_pose_ << std::endl;
    // std::cout << "goal t " << goal_p_unscaled_ <<std::endl; 

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

void PolynomialTrajectoryGenerator::construct_line_(int steps)
{
    // std::cout << "Constructing Line" << std::endl;
    // std::cout << "start" << start_p_unscaled_ << std::endl;
    // std::cout << "end" << goal_p_unscaled_ << std::endl;
    // coeff * A = b 
    Eigen::MatrixXd b(goal_p_unscaled_.size(), 6);
    Eigen::MatrixXd coeff(goal_p_unscaled_.size(), order_+1);

    b << 
        start_p_unscaled_,
        goal_p_unscaled_,
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

Eigen::MatrixXd 
PolynomialTrajectoryGenerator::diff_(int pol_order, int diff_order) 
{
    if (diff_order==0) return Eigen::MatrixXd::Identity(pol_order+1, pol_order+1);

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(pol_order+1, pol_order);
    for (int i = 1; i<= pol_order; ++i) result(i, i-1) = i;

    if (diff_order==1) return result;

    return result*diff_(pol_order-1, diff_order-1);
}

std::vector<Eigen::VectorXd>
PolynomialTrajectoryGenerator::evaluate_polynomial(const Eigen::VectorXd& t,
                    const Eigen::MatrixXd& coeff)
{
    const int steps = t.size();
    const int degree = coeff.cols() - 1;

    Eigen::MatrixXd powers = generate_pow_vector_(t, degree);
    Eigen::MatrixXd values = coeff * powers;

    std::vector<Eigen::VectorXd> result;
    for (int i = 0; i < steps; ++i) {
        result.push_back(values.col(i));
    }

    return result;
}

Eigen::VectorXd 
PolynomialTrajectoryGenerator::generate_pow_vector_(double s, int pow) {
    Eigen::VectorXd vec(pow + 1);
    for (int i = 0; i <= pow; ++i) {
        vec(i) = std::pow(s, i);
    }
    return vec;
}

Eigen::MatrixXd 
PolynomialTrajectoryGenerator::generate_pow_vector_(const Eigen::VectorXd& s, int pow)
{
    const int n = s.size();
    Eigen::MatrixXd mat(pow + 1, n);

    mat.row(0).setOnes();
    if (pow == 0) {
        return mat;
    }

    mat.row(1) = s.transpose();

    for (int i = 2; i <= pow; ++i) {
        mat.row(i) = mat.row(i - 1).array() * s.transpose().array();
    }

    return mat;
}

Eigen::VectorXd PolynomialTrajectoryGenerator::vee(const Eigen::MatrixXd m){
    Eigen::MatrixXd symm = 0.5*(m - m.transpose());
    Eigen::VectorXd v(2*(m.rows()-1));
    v << m(0,3),
         m(1,3),
         m(2,3),
         symm(2,1),
         symm(0,2),
         symm(1,0);

    return v;
}

Eigen::MatrixXd PolynomialTrajectoryGenerator::hat(const Eigen::VectorXd v){
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(4,4);
    M(0, 1) = -v(5);
    M(0, 2) =  v(4);
    M(1, 0) =  v(5);
    M(1, 2) = -v(3);
    M(2, 0) = -v(4);
    M(2, 1) =  v(3);
    M(0, 3) = v(0);
    M(1, 3) = v(1);
    M(2, 3) = v(2);

    return M;
}





} // namespace MP