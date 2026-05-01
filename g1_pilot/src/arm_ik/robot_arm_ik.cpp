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
// HumanoidIK Implementation
// ============================================================================

namespace ArmPilot {

HumanoidIK::HumanoidIK(
    pinocchio::Model& model,
    pinocchio::GeometryModel& geom_model
) : model_(model), geom_model_(geom_model)
{
    // Get required joint IDs    
    L_hand_id_ = model_.getFrameId("L_ee");
    R_hand_id_ = model_.getFrameId("R_ee");
    // oMcamera = model_.getFrameId("d435_link", pinocchio::BODY);
    // oMLidar = model_.getFrameId("mid360_link", pinocchio::BODY);

    // Setup Collision
    geom_model_.addAllCollisionPairs();    
    filter_adjacent_collision_pairs();

    // Setup IK problem
    setup_optimization();

    // Initialize data structures
    data_ = pinocchio::Data(model_);
    geom_data_ = pinocchio::GeometryData(geom_model_);
    init_data_ = Eigen::VectorXd::Zero(model_.nq);
    
    // Initialize filter and data
    Eigen::VectorXd weights(4);
    weights << 0.35, 0.3, 0.25, 0.1;
    smooth_filter_ = std::make_unique<WeightedMovingFilter>(weights, model_.nv);

    nq_ = model_.nq;
    nv_ = model_.nv;

}

void HumanoidIK::filter_adjacent_collision_pairs() {
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

void HumanoidIK::setup_optimization() {

    pinocchio::ModelTpl<casadi::SX> cmodel(model_.cast<casadi::SX>());
    pinocchio::DataTpl<casadi::SX> cdata(cmodel);

    casadi::SX cq = casadi::SX::sym("q", cmodel.nq);
    casadi::SX cq_last = casadi::SX::sym("q", cmodel.nq);
    casadi::SX cTf_l = casadi::SX::sym("tf_l", 4, 4);
    casadi::SX cTf_r = casadi::SX::sym("tf_r", 4, 4);

    Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> cq_eigen(cmodel.nq);
    Eigen::Matrix<casadi::SX,4,4>  Target_L_eig;
    Eigen::Matrix<casadi::SX,4,4>  Target_R_eig;

    pinocchio::casadi::copy(cq, cq_eigen);
    pinocchio::casadi::copy(cTf_l, Target_L_eig);
    pinocchio::casadi::copy(cTf_r, Target_R_eig);

    auto L_hand_id = cmodel.getFrameId("L_ee");
    auto R_hand_id = cmodel.getFrameId("R_ee");

    pinocchio::framesForwardKinematics(cmodel, cdata, cq_eigen);
    pinocchio::SE3Tpl<casadi::SX> T_L = cdata.oMf[L_hand_id];
    pinocchio::SE3Tpl<casadi::SX> T_R = cdata.oMf[R_hand_id];

    pinocchio::SE3Tpl<casadi::SX>  Target_L(Target_L_eig);
    pinocchio::SE3Tpl<casadi::SX>  Target_R(Target_R_eig);

    casadi::SX Err_L(casadi::Sparsity::dense(6,1));
    casadi::SX Err_R(casadi::Sparsity::dense(6,1));

    pinocchio::casadi::copy(
        pinocchio::log6(T_L.actInv(Target_L)).toVector(),
        Err_L
    );

    pinocchio::casadi::copy(
        pinocchio::log6(T_R.actInv(Target_R)).toVector(),
        Err_R
    );

    casadi::SX left_trans_err = Err_L(casadi::Slice(0,3),0);
    casadi::SX right_trans_err = Err_R(casadi::Slice(0,3),0);
    casadi::SX left_rot_err = Err_L(casadi::Slice(3,6),0);
    casadi::SX right_rot_err = Err_R(casadi::Slice(3,6),0);

    casadi::SX both_arm_cost = (
        50.0 * (
            casadi::SX::sumsqr(left_trans_err) +
            casadi::SX::sumsqr(right_trans_err)
        ) + 
        3.0 * (
            casadi::SX::sumsqr(left_rot_err) +
            casadi::SX::sumsqr(right_rot_err)
        ) + 
        0.005 * casadi::SX::sumsqr(cq) +
        0.1 * casadi::SX::sumsqr(cq - cq_last)
    );

    casadi::SX left_arm_cost = (
        50.0 * casadi::SX::sumsqr(left_trans_err) + 
        3.0 * casadi::SX::sumsqr(left_rot_err) + 
        0.005 * casadi::SX::sumsqr(cq) +
        0.1 * casadi::SX::sumsqr(cq - cq_last)
    );

    casadi::SX right_arm_cost = (
        50.0 * casadi::SX::sumsqr(right_trans_err) + 
        3.0 * casadi::SX::sumsqr(right_rot_err) + 
        0.005 * casadi::SX::sumsqr(cq) +
        0.1 * casadi::SX::sumsqr(cq - cq_last)
    );

    // Functions
    casadi::Function both_arm_cost_f = casadi::Function(
        "both_arm_cost",
        {cq, cq_last, cTf_l, cTf_r},
        {both_arm_cost}
    );
    casadi::Function left_arm_cost_f = casadi::Function(
        "left_arm_cost",
        {cq, cq_last, cTf_l},
        {left_arm_cost}
    );
    casadi::Function right_arm_cost_f = casadi::Function(
        "right_arm_cost",
        {cq, cq_last, cTf_r},
        {right_arm_cost}
    );

    b_var_q_ = opti_.variable(model_.nq, 1);
    b_var_q_last_ = opti_.parameter(model_.nq, 1);
    b_param_tf_l_ = opti_.parameter(4, 4);
    b_param_tf_r_ = opti_.parameter(4, 4);

    l_var_q_ = opti_l_.variable(model_.nq, 1);
    l_var_q_last_ = opti_l_.parameter(model_.nq, 1);
    l_param_tf_l_ = opti_l_.parameter(4, 4);

    r_var_q_ = opti_r_.variable(model_.nq, 1);
    r_var_q_last_ = opti_r_.parameter(model_.nq, 1);
    r_param_tf_r_ = opti_r_.parameter(4, 4);
            
    casadi::DM lower_limits = eigen_to_casadi(model_.lowerPositionLimit);
    casadi::DM upper_limits = eigen_to_casadi(model_.upperPositionLimit);

    if (cmodel.existJointName("waist_yaw_joint")){
        int idx = cmodel.joints[cmodel.getJointId("waist_yaw_joint")].idx_q();
        lower_limits(idx) = -0.0001;
        upper_limits(idx) = 0.0001;
    }

    if (cmodel.existJointName("waist_pitch_joint")){
        int idx = cmodel.joints[cmodel.getJointId("waist_pitch_joint")].idx_q();
        lower_limits(idx) = -0.0001;
        upper_limits(idx) = 0.0001;
    }

    if (cmodel.existJointName("waist_roll_joint")){
        int idx = cmodel.joints[cmodel.getJointId("waist_roll_joint")].idx_q();
        lower_limits(idx) = -0.0001;
        upper_limits(idx) = 0.0001;
    }

    opti_.subject_to(opti_.bounded(lower_limits, b_var_q_, upper_limits));
    opti_l_.subject_to(opti_l_.bounded(lower_limits, l_var_q_, upper_limits));
    opti_r_.subject_to(opti_r_.bounded(lower_limits, r_var_q_, upper_limits));

    // Minimize
    opti_.minimize(both_arm_cost_f({
        b_var_q_, b_var_q_last_, b_param_tf_l_, b_param_tf_r_
    })[0]);
    opti_l_.minimize(left_arm_cost_f({
        l_var_q_, l_var_q_last_, l_param_tf_l_
    })[0]);
    opti_r_.minimize(right_arm_cost_f({
        r_var_q_, r_var_q_last_, r_param_tf_r_
    })[0]);
    
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
    opti_l_.solver("ipopt", opts);
    opti_r_.solver("ipopt", opts);

}

void HumanoidIK::reset() {
    init_data_ = Eigen::VectorXd::Zero(model_.nq);
    smooth_filter_->reset();
}

std::pair<double, double> HumanoidIK::compute_error(Eigen::Matrix4d A, Eigen::Matrix4d B){
    return {
        (A.block<3,1>(0,3) - A.block<3,1>(0,3)).norm(),
        std::acos(std::clamp(
            0.5 * ((
                A.block<3, 3>(0, 0) * B.block<3, 3>(0, 0).transpose()
            ).trace() - 1),
            -1.0,1.0
        ))
    };
}

JointState HumanoidIK::solve_ik(
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

    // Set optimization initial guess and parameters
    opti_.set_initial(b_var_q_, eigen_to_casadi(init_data_));
    opti_.set_value(b_param_tf_l_, eigen_to_casadi(left_wrist));
    opti_.set_value(b_param_tf_r_, eigen_to_casadi(right_wrist));
    opti_.set_value(b_var_q_last_, eigen_to_casadi(init_data_));

    // Solve optimization problem
    std::vector<double> sol_q_vec;

    try {

        // Solve optimization problem
        casadi::OptiSol sol = opti_.solve();

        // Extract solution
        sol_q_vec = static_cast<std::vector<double>>(sol.value(b_var_q_));

    } catch (const std::exception& e) {
        std::cerr << "ERROR in convergence: " << e.what() << std::endl;
        
        // Get debug solution
        sol_q_vec = static_cast<std::vector<double>>(
            opti_.debug().value(b_var_q_)
        );

    }

    sol_q = Eigen::Map<Eigen::VectorXd>(sol_q_vec.data(), model_.nq);

    pinocchio::framesForwardKinematics(model_, data_, sol_q);

    auto [l_pos_err, l_rot_err] = compute_error(data_.oMf[L_hand_id_].toHomogeneousMatrix(), left_wrist);
    std::cout << "IK result: left_wrist: pos_err: " << l_pos_err << ", rot_err: " << l_rot_err << std::endl;
    auto [r_pos_err, r_rot_err] = compute_error(data_.oMf[R_hand_id_].toHomogeneousMatrix(), right_wrist);
    std::cout << "IK result: right_wrist: pos_err: " << r_pos_err << ", rot_err: " << r_rot_err << std::endl;
    
    // Apply smoothing filter
    // smooth_filter_->add_data(sol_q);
    // sol_q = smooth_filter_->filtered_data();
    
    // Compute velocity
    sol_v = Eigen::VectorXd::Zero(model_.nv);
    
    // Compute feedforward torques using RNEA
    sol_t = pinocchio::rnea(
        model_, data_, sol_q, sol_v,
        Eigen::VectorXd::Zero(model_.nv)
    );

    generalize_ext_wrenches(EE_efrc_L, EE_efrc_R);

    JointState result = vectors_to_jointstate(
        sol_q, sol_v, sol_t, model_
    );
    init_data_ = sol_q;
    
    return result;

}

JointState HumanoidIK::solve_ik(
    const Eigen::Matrix4d& wrist,
    const bool left,
    const Eigen::VectorXd* current_lr_arm_motor_q,
    const Eigen::VectorXd* current_lr_arm_motor_dq,
    const Eigen::VectorXd* EE_efrc
) 
{
    // Update initial guess
    if (current_lr_arm_motor_q != nullptr) {
        init_data_ = *current_lr_arm_motor_q;
    }

    if (left){
        opti_l_.set_initial(l_var_q_, eigen_to_casadi(init_data_));
        opti_l_.set_value(l_param_tf_l_, eigen_to_casadi(wrist));
        opti_l_.set_value(l_var_q_last_, eigen_to_casadi(init_data_));
    } else {
        opti_r_.set_initial(r_var_q_, eigen_to_casadi(init_data_));
        opti_r_.set_value(r_param_tf_r_, eigen_to_casadi(wrist));
        opti_r_.set_value(r_var_q_last_, eigen_to_casadi(init_data_));
    }

    // Solve optimization problem
    std::vector<double> sol_q_vec;
    
    try {
        if (left){
            casadi::OptiSol sol = opti_l_.solve();
            sol_q_vec = static_cast<std::vector<double>>(sol.value(l_var_q_));
        } else {
            casadi::OptiSol sol = opti_r_.solve();
            sol_q_vec = static_cast<std::vector<double>>(sol.value(r_var_q_));
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR in convergence: " << e.what() << std::endl;
        
        // Get debug solution
        if (left){
            sol_q_vec = static_cast<std::vector<double>>(
                opti_.debug().value(l_var_q_)
            );
        } else {
            sol_q_vec = static_cast<std::vector<double>>(
                opti_.debug().value(r_var_q_)
            );
        }
        
    }

    sol_q = Eigen::Map<Eigen::VectorXd>(sol_q_vec.data(), model_.nq);
    
    // // Apply smoothing filter
    // smooth_filter_->add_data(sol_q);
    // sol_q = smooth_filter_->filtered_data();

    pinocchio::framesForwardKinematics(model_, data_, sol_q);

    if (left){
        auto [l_pos_err, l_rot_err] = compute_error(data_.oMf[L_hand_id_].toHomogeneousMatrix(), left_wrist);
        std::cout << "IK result: left_wrist: pos_err: " << l_pos_err << ", rot_err: " << l_rot_err << std::endl;
    } else {
        auto [r_pos_err, r_rot_err] = compute_error(data_.oMf[R_hand_id_].toHomogeneousMatrix(), right_wrist);
        std::cout << "IK result: right_wrist: pos_err: " << r_pos_err << ", rot_err: " << r_rot_err << std::endl;
    }
    
    // Compute velocity
    sol_v = Eigen::VectorXd::Zero(model_.nv);
    
    // Compute feedforward torques using RNEA
    sol_t = pinocchio::rnea(
        model_, data_, sol_q, sol_v,
        Eigen::VectorXd::Zero(model_.nv)
    );

    if (left) generalize_ext_wrenches(EE_efrc, nullptr);
    else generalize_ext_wrenches(nullptr, EE_efrc);

    JointState result = vectors_to_jointstate(
        sol_q, sol_v, sol_t, model_
    );
    init_data_ = sol_q;
    
    return result;

}

void HumanoidIK::generalize_ext_wrenches(
    const Eigen::VectorXd* EE_efrc_L,
    const Eigen::VectorXd* EE_efrc_R
){
    if (EE_efrc_L == nullptr && EE_efrc_R == nullptr)
        return;

    if (EE_efrc_L != nullptr){

        J_L = pinocchio::Data::Matrix6x::Zero(6, nv_);

        pinocchio::computeFrameJacobian(
            model_, data_, sol_q, 
            L_hand_id_, pinocchio::LOCAL_WORLD_ALIGNED, 
            J_L
        );

        sol_t = J_L.transpose() * (*EE_efrc_L);
    }

    if (EE_efrc_R != nullptr){

        J_R = pinocchio::Data::Matrix6x::Zero(6, nv_);

        pinocchio::computeFrameJacobian(
            model_, data_, sol_q, 
            R_hand_id_, pinocchio::LOCAL_WORLD_ALIGNED, 
            J_R
        );

        sol_t = J_R.transpose() * (*EE_efrc_R);
    }

    

}

} // ArmPilot namespace
