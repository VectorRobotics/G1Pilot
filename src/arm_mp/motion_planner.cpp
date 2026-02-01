#include "arm_mp/motion_planner.h"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <iomanip>


namespace MP{

EE_MotionPlanner::EE_MotionPlanner(int order) : 
    order_(order) {


}
EE_MotionPlanner::~EE_MotionPlanner() {}

// std::vector<Eigen::Matrix4d> EE_MotionPlanner::planTrajectory(
//     const Eigen::Matrix4d* start_pose,
//     const Eigen::Matrix4d* goal_pose,
//     double duration,
//     double time_step,
//     const Eigen::Vector3d* start_vel,
//     const Eigen::Vector3d* goal_vel,
//     const Eigen::Vector3d* start_acc,
//     const Eigen::Vector3d* goal_acc,
//     const double MAX_LIN_VEL,
//     const double MAX_ANG_VEL,
//     const double MAX_LIN_ACC,
//     const double MAX_ANG_ACC
// ) {
//     int steps = ceil(duration/time_step);

//     std::vector<Eigen::Matrix4d> path = planPath(
//                                             start_pose,
//                                             goal_pose,
//                                             start_vel,
//                                             goal_vel,
//                                             start_acc,
//                                             goal_acc,
//                                             steps
//                                         );



// }

// std::vector<Eigen::Matrix4d> EE_MotionPlanner::planPath(
//     const Eigen::Matrix4d* start_pose,
//     const Eigen::Matrix4d* goal_pose,
//     const Eigen::Vector3d* start_vel,
//     const Eigen::Vector3d* goal_vel,
//     const Eigen::Vector3d* start_acc,
//     const Eigen::Vector3d* goal_acc,
//     int steps
// ) {
//     Eigen::Matrix3d start_R = start_pose->block<3,3>(0, 0);
//     Eigen::Matrix3d goal_R = goal_pose->block<3,3>(0, 0);
//     Eigen::Vector3d start_p = start_pose->block<3,1>(0, 3);
//     Eigen::Vector3d goal_p = goal_pose->block<3,1>(0, 3);

//     std::vector<Eigen::VectorXd> line = construct_line_(
//         start_p, goal_p, start_vel, goal_vel, start_acc, goal_acc, steps
//     )


//     std::vector<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>


// }

std::vector<Eigen::VectorXd> EE_MotionPlanner::construct_line_(
    Eigen::Vector3d start_p,
    Eigen::Vector3d goal_p,
    const Eigen::Vector3d* start_vel,
    const Eigen::Vector3d* goal_vel,
    const Eigen::Vector3d* start_acc,
    const Eigen::Vector3d* goal_acc,
    int steps
){
    std::vector<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> constraints;

    constraints.push_back({
            generate_pow_vector_(0.0, order_),
            start_p
    });
    constraints.push_back({
            generate_pow_vector_(1.0, order_),
            goal_p
    });

    if (start_vel!=nullptr) {
        constraints.push_back({
            diff_(order_) * generate_pow_vector_(0.0, order_ - 1),
            *start_vel
        });
    }

    if (goal_vel!=nullptr) {
        constraints.push_back({
            diff_(order_) * generate_pow_vector_(1.0, order_ - 1),
            *goal_vel
        });
    }

    if (start_acc!=nullptr) {
        constraints.push_back({
            diff_(order_, 2) * generate_pow_vector_(0.0, order_ - 2),
            *start_acc
        });
    }

    if (goal_acc!=nullptr) {
        constraints.push_back({
            diff_(order_, 2) * generate_pow_vector_(1.0, order_ - 2),
            *goal_acc
        });
    }

    // coeff * A = b 
    Eigen::MatrixXd A(order_ + 1, constraints.size());
    Eigen::MatrixXd b(3, constraints.size());
    Eigen::MatrixXd coeff(3, order_+1);

    for (size_t i = 0; i < constraints.size(); ++i) {
        A.col(i) = constraints[i].first;
        b.col(i) = constraints[i].second;
    }

    // std::cout << "A: \n" << A << std::endl;
    // std::cout << "b: \n" << b << std::endl;

    assert(b.cols() == A.cols());

    if (constraints.size()>order_+1) {

        Eigen::MatrixXd AtA = A * A.transpose();
        coeff = b * A.transpose().llt().solve(AtA);
    } else {

        Eigen::MatrixXd AtA = A.transpose() * A;
        coeff = b * AtA.llt().solve(A.transpose());
    }

    // std::cout << std::fixed << std::setprecision(2);
    // std::cout << "coeff: \n" << coeff << std::endl;

    Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(steps, 0, 1);

    std::vector<Eigen::VectorXd> line = evaluate_polynomial(t, coeff);

    return line;
}

Eigen::MatrixXd EE_MotionPlanner::diff_(int pol_order, int diff_order) 
{
    if (diff_order==0) return Eigen::MatrixXd::Identity(pol_order+1, pol_order+1);

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(pol_order+1, pol_order);
    for (int i = 1; i<= pol_order; ++i) result(i, i-1) = i;

    if (diff_order==1) return result;

    return result*diff_(pol_order-1, diff_order-1);
}

std::vector<Eigen::VectorXd>
EE_MotionPlanner::evaluate_polynomial(const Eigen::VectorXd& t,
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

Eigen::VectorXd EE_MotionPlanner::generate_pow_vector_(double s, int pow) {
    Eigen::VectorXd vec(pow + 1);
    for (int i = 0; i <= pow; ++i) {
        vec(i) = std::pow(s, i);
    }
    return vec;
}

Eigen::MatrixXd EE_MotionPlanner::generate_pow_vector_(const Eigen::VectorXd& s, int pow)
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



} // namespace MP