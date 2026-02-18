#include "arm_ik/robot_arm_ik.h"

#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <memory>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <cmath>

// ============================================================================
// G1_29_ArmIK Implementation
// ============================================================================

namespace ArmPilot {

G1_29_ArmIK::G1_29_ArmIK(
    pinocchio::Model& model,
    pinocchio::GeometryModel& geom_model
) : model_(model), geom_model_(geom_model)
{
    std::cout << std::fixed << std::setprecision(5);
    
    L_hand_id_ = model_.getFrameId("L_ee");
    R_hand_id_ = model_.getFrameId("R_ee");
    oMcamera = model_.getFrameId("d435_link", pinocchio::BODY);
    oMLidar = model_.getFrameId("mid360_link", pinocchio::BODY);
    
    setup_optimization();
    initialize_collision_model();

    data_ = pinocchio::Data(model_);
    geom_data_ = pinocchio::GeometryData(geom_model_);
    init_data_ = Eigen::VectorXd::Zero(model_.nq);
    
    // Initialize filter and data
    Eigen::VectorXd weights(4);
    weights << 0.4, 0.3, 0.2, 0.1;
    smooth_filter_ = std::make_unique<WeightedMovingFilter>(weights, 14);

    nq_ = model_.nq;
    nv_ = model_.nv;

}

void G1_29_ArmIK::initialize_collision_model() {
    
    // Add all collision pairs
    geom_model_.addAllCollisionPairs();
    
    // Filter out adjacent link collisions
    filter_adjacent_collision_pairs();
    
    std::cout << "num collision pairs - initial: " 
              << geom_model_.collisionPairs.size() << std::endl;
    std::cout << "Number of geometry objects: " 
              << geom_model_.geometryObjects.size() << std::endl;
}

void G1_29_ArmIK::filter_adjacent_collision_pairs() {
    // Get kinematic adjacency from the model
    std::set<std::pair<int, int>> adjacent_pairs;
    for (int i = 1; i < model_.njoints; ++i) {
        adjacent_pairs.insert({model_.parents[i], i});
        adjacent_pairs.insert({i, model_.parents[i]});
    }
    
    // Filter out neighboring links
    std::vector<pinocchio::CollisionPair> filtered_pairs;
    for (const auto& cp : geom_model_.collisionPairs) {
        int link1 = geom_model_.geometryObjects[cp.first].parentJoint;
        int link2 = geom_model_.geometryObjects[cp.second].parentJoint;
        
        if (adjacent_pairs.find({link1, link2}) == adjacent_pairs.end()) {
            filtered_pairs.push_back(cp);
        }
    }
    
    geom_model_.collisionPairs = filtered_pairs;
}

void G1_29_ArmIK::setup_optimization() {
    #ifdef USE_CASADI
    // Create optimization variables and parameters
    var_q_ = opti_.variable(model_.nq, 1);
    var_q_last_ = opti_.parameter(model_.nq, 1);
    param_tf_l_ = opti_.parameter(4, 4);
    param_tf_r_ = opti_.parameter(4, 4);
    
    // Set joint limits as constraints
    casadi::DM lower_limits = eigen_to_casadi(model_.lowerPositionLimit);
    casadi::DM upper_limits = eigen_to_casadi(model_.upperPositionLimit);
    opti_.subject_to(opti_.bounded(lower_limits, var_q_, upper_limits));

    pinocchio::ModelTpl<casadi::SX> cmodel(model_.cast<casadi::SX>());
    pinocchio::DataTpl<casadi::SX> cdata(cmodel);

    auto L_hand_id = cmodel.getFrameId("L_ee");
    auto R_hand_id = cmodel.getFrameId("R_ee");

    casadi::SX cq = casadi::SX::sym("q", cmodel.nq);
    casadi::SX cTf_l = casadi::SX::sym("tf_l", 4, 4);
    casadi::SX cTf_r = casadi::SX::sym("tf_r", 4, 4);

    Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> cq_eigen(cmodel.nq);
    for (int i = 0; i < cmodel.nq; ++i) {
        cq_eigen(i) = cq(i);
    }

    pinocchio::framesForwardKinematics(cmodel, cdata, cq_eigen);
    pinocchio::SE3Tpl<casadi::SX> T_L_se3 = cdata.oMf[L_hand_id];
    pinocchio::SE3Tpl<casadi::SX> T_R_se3 = cdata.oMf[R_hand_id];

    casadi::SX rot_L(casadi::Sparsity::dense(3,3));
    casadi::SX rot_R(casadi::Sparsity::dense(3,3));
    casadi::SX trans_L(casadi::Sparsity::dense(3,1));
    casadi::SX trans_R(casadi::Sparsity::dense(3,1));
    casadi::SX target_rot_L(casadi::Sparsity::dense(3,3));
    casadi::SX target_rot_R(casadi::Sparsity::dense(3,3));
    casadi::SX target_trans_L(casadi::Sparsity::dense(3,1));
    casadi::SX target_trans_R(casadi::Sparsity::dense(3,1));
    Eigen::Matrix<casadi::SX, 3, 3> rot_err_mat_L;
    Eigen::Matrix<casadi::SX, 3, 3> rot_err_mat_R;
    casadi::SX rot_err_L(casadi::Sparsity::dense(3,1));
    casadi::SX rot_err_R(casadi::Sparsity::dense(3,1));
    casadi::SX trans_err_L(casadi::Sparsity::dense(3,1));
    casadi::SX trans_err_R(casadi::Sparsity::dense(3,1));

    pinocchio::casadi::copy(T_L_se3.rotation(), rot_L);
    pinocchio::casadi::copy(T_R_se3.rotation(), rot_R);
    pinocchio::casadi::copy(T_L_se3.translation(), trans_L);
    pinocchio::casadi::copy(T_R_se3.translation(), trans_R);

    target_rot_L = cTf_l(casadi::Slice(0,3), casadi::Slice(0,3));;
    target_rot_R = cTf_r(casadi::Slice(0,3), casadi::Slice(0,3));;
    target_trans_L = cTf_l(casadi::Slice(0,3), 3);
    target_trans_R = cTf_r(casadi::Slice(0,3), 3);

    pinocchio::casadi::copy(casadi::SX::mtimes(rot_L, target_rot_L.T()), rot_err_mat_L);
    pinocchio::casadi::copy(casadi::SX::mtimes(rot_R, target_rot_R.T()), rot_err_mat_R);

    pinocchio::casadi::copy(pinocchio::log3(rot_err_mat_L), rot_err_L);
    pinocchio::casadi::copy(pinocchio::log3(rot_err_mat_R), rot_err_R);
    trans_err_L = trans_L - target_trans_L;
    trans_err_R = trans_R - target_trans_R;

    // Translation Error
    casadi::SX trans_err = casadi::SX::vertcat({
        trans_err_L,
        trans_err_R
    });
    casadi::Function translational_error = casadi::Function("trans_err", {cq, cTf_l, cTf_r}, {trans_err});

    // Rotational Error (using log3 for SO3 distance)
    casadi::SX rot_err = casadi::SX::vertcat({
        rot_err_L,
        rot_err_R
    });
    casadi::Function rotational_error = casadi::Function("rot_err", {cq, cTf_l, cTf_r}, {rot_err});

    // Costs
    casadi::MX cost_trans = casadi::MX::sumsqr(translational_error({var_q_, param_tf_l_, param_tf_r_})[0]);
    casadi::MX cost_rot = casadi::MX::sumsqr(rotational_error({var_q_, param_tf_l_, param_tf_r_})[0]);
    casadi::MX cost_reg = casadi::MX::sumsqr(var_q_);
    casadi::MX cost_smooth = casadi::MX::sumsqr(var_q_ - var_q_last_);

    opti_.minimize(50.0 * cost_trans + cost_rot + 0.02 * cost_reg + 0.1 * cost_smooth);
    
    // Set optimization options
    casadi::Dict opts;
    opts["expand"] = true;
    opts["detect_simple_bounds"] = true;
    opts["calc_lam_p"] = false;
    opts["print_time"] = false;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.print_level"] = 0;
    opts["ipopt.max_iter"] = 50;
    opts["ipopt.tol"] = 1e-6;
    opts["ipopt.acceptable_tol"] = 5e-4;
    opts["ipopt.acceptable_iter"] = 5;
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.derivative_test"] = "none";
    opts["ipopt.jacobian_approximation"] = "exact";
    
    opti_.solver("ipopt", opts);
    #endif // USE_CASADI
}

JointState G1_29_ArmIK::solve_ik(
    const Eigen::Matrix4d& left_wrist,
    const Eigen::Matrix4d& right_wrist,
    const Eigen::VectorXd* current_lr_arm_motor_q,
    const Eigen::VectorXd* current_lr_arm_motor_dq,
    const Eigen::VectorXd* EE_efrc_L,
    const Eigen::VectorXd* EE_efrc_R
) 
{
    // Update initial guess
    if (current_lr_arm_motor_q != nullptr) {
        init_data_ = *current_lr_arm_motor_q;
    }

    // #TODO: Get current camera/lidar frame to get reference for desired transforms
    #ifdef USE_CASADI
        
        // Set optimization initial guess and parameters
        opti_.set_initial(var_q_, eigen_to_casadi(init_data_));
        opti_.set_value(param_tf_l_, eigen_to_casadi(left_wrist));
        opti_.set_value(param_tf_r_, eigen_to_casadi(right_wrist));
        opti_.set_value(var_q_last_, eigen_to_casadi(init_data_));

    
    #else // USE_CASADI

        var_q_ = init_data_;
        param_tf_l_ = eigen_to_pinocchio(left_wrist);
        param_tf_r_ = eigen_to_pinocchio(right_wrist);
        var_q_last_ = init_data_;


        Eigen::VectorXd v_itr(model_.nv);
        pinocchio::Data::Matrix6x J_left(6,model_.nv);
        pinocchio::Data::Matrix6x J_right(6,model_.nv);
        pinocchio::Data::MatrixXs J(12,model_.nv);
        J_left.setZero();
        J_right.setZero();

        // Frames for camera and lidar
        // data_.oMf[oMcamera];
        // data_.oMf[oMLidar];

    #endif // USE_CASADI

    try {

        #ifdef USE_CASADI

            // Solve optimization problem
            casadi::OptiSol sol = opti_.solve();
            
            // Extract solution
            std::vector<double> sol_q_vec = static_cast<std::vector<double>>(sol.value(var_q_));

        #else // USE_CASADI
            // Using interation method
            for (int i=0;;i++)
            {
                pinocchio::forwardKinematics(model_,data_,var_q_);
                const pinocchio::SE3 left_dMi = param_tf_l_.actInv(data_.oMi[left_wrist_id_]);
                const pinocchio::SE3 right_dMi = param_tf_r_.actInv(data_.oMi[right_wrist_id_]);
                err.head<6>() = pinocchio::log6(left_dMi).toVector();
                err.tail<6>() = pinocchio::log6(right_dMi).toVector();

                if(err.norm() < eps) {
                    break;
                }
                if (i >= IT_MAX) {
                    throw std::runtime_error("Maximum iterations reached without convergence.");
                }
                pinocchio::computeJointJacobian(model_,data_,var_q_,left_wrist_id_,J_left);
                pinocchio::computeJointJacobian(model_,data_,var_q_,right_wrist_id_,J_right);
                J.topRows<6>() = J_left;
                J.bottomRows<6>() = J_right;
                pinocchio::Data::MatrixXs JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += damp;
                v_itr.noalias() = - J.transpose() * JJt.ldlt().solve(err);
                var_q_ = pinocchio::integrate(model_,var_q_,v_itr * DT);
            }

            var_q_last_ = var_q_;

            // Extract solution
            std::vector<double> sol_q_vec(var_q_.data(), var_q_.data() + var_q_.size());
        #endif // USE_CASADI

        Eigen::VectorXd sol_q = Eigen::Map<Eigen::VectorXd>(sol_q_vec.data(), model_.nq);
        
        // Apply smoothing filter
        smooth_filter_->add_data(sol_q);
        sol_q = smooth_filter_->filtered_data();
        
        // Compute velocity
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            v = (*current_lr_arm_motor_dq) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        // Compute feedforward torques using RNEA
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            model_, data_, sol_q, v,
            Eigen::VectorXd::Zero(model_.nv)
        );

        // Add external forces if provided
        if (EE_efrc_L != nullptr && EE_efrc_R != nullptr) {
            // Compute Jacobians for both end effectors
            pinocchio::Data::Matrix6x J_L(6, nv_);
            pinocchio::Data::Matrix6x J_R(6, nv_);
            
            pinocchio::computeFrameJacobian(model_, data_, sol_q, 
                                           L_hand_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_L);
            pinocchio::computeFrameJacobian(model_, data_, sol_q, 
                                           R_hand_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_R);
            
            // Compute torques from external forces
            Eigen::VectorXd tau_ext_L = J_L.transpose() * (*EE_efrc_L);
            Eigen::VectorXd tau_ext_R = J_R.transpose() * (*EE_efrc_R);
            
            // Combine external torques (first 4 from left, last 4 from right)
            Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(nv_);
            if (nv_ >= 8) {
                tau_ext.head(4) = tau_ext_L.head(4);
                tau_ext.tail(4) = tau_ext_R.tail(4);
            }
            
            sol_tauff += tau_ext;
        }

        JointState result = vectors_to_jointstate(
            sol_q, v, sol_tauff, model_
        );
        
        return result;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR in convergence: " << e.what() << std::endl;
        
        #ifdef USE_CASADI
        // Get debug solution
        std::vector<double> sol_q_vec = static_cast<std::vector<double>>(
            opti_.debug().value(var_q_));
        #else // USE_CASADI
        // Get debug solution
        std::vector<double> sol_q_vec(var_q_.data(), var_q_.data() + var_q_.size());
        #endif // USE_CASADI

        Eigen::VectorXd sol_q = Eigen::Map<Eigen::VectorXd>(
            sol_q_vec.data(), model_.nq);
        
        smooth_filter_->add_data(sol_q);
        sol_q = smooth_filter_->filtered_data();
        
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            v = (*current_lr_arm_motor_dq) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            model_, data_, sol_q, v,
            Eigen::VectorXd::Zero(model_.nv)
        );
        
        std::cerr << "sol_q: " << sol_q.transpose() << std::endl;
        std::cerr << "left_pose:\n" << left_wrist << std::endl;
        std::cerr << "right_pose:\n" << right_wrist << std::endl;

        JointState result = vectors_to_jointstate(
            sol_q, v, sol_tauff, model_
        );
        
        return result;
    }
}

} // ArmPilot namespace
