#ifndef CASADI_EIGEN_UTILS_H
#define CASADI_EIGEN_UTILS_H

#ifdef USE_CASADI

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <vector>

#include <pinocchio/spatial/se3.hpp>

/**
 * @brief Convert Eigen::VectorXd to casadi::DM
 */
inline casadi::DM eigen_to_casadi(const Eigen::VectorXd& vec) {
    std::vector<double> data(vec.data(), vec.data() + vec.size());
    return casadi::DM(data);
}

/**
 * @brief Convert Eigen::Matrix4d to casadi::DM
 */
inline casadi::DM eigen_to_casadi(const Eigen::Matrix4d& mat) {
    std::vector<double> data;
    data.reserve(16);
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            data.push_back(mat(row, col));
        }
    }
    return casadi::DM::reshape(casadi::DM(data), 4, 4);
}

inline pinocchio::SE3 eigen_to_pinocchio(const Eigen::Matrix4d& mat) {
    return pinocchio::SE3(mat);
}



#endif // USE_CASADI
#endif // CASADI_EIGEN_UTILS_H