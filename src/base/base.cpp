#include "base/base.h"

#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>


#include "arm_control/arm_control.h"
#include "arm_ik/arm_ik.h"
#include "arm_mp/arm_mp.h"

namespace ArmPilot {

G1DualArm::G1DualArm(
    const RobotConfig* robot_config
)
{
    robot_config_ = RobotConfig();
    if (robot_config != nullptr) {
        robot_config_ = *robot_config;
    }

    // (1) Basic Model
    pinocchio::urdf::buildModel(robot_config_.asset_file, robot_model_);
    pinocchio::urdf::buildGeom(
        robot_model_, 
        robot_config_.asset_file, 
        pinocchio::COLLISION, 
        geom_robot_model_, 
        robot_config_.asset_root
    );
    add_end_effector_frames();
    robot_data_ = pinocchio::Data(robot_model_);
    reference_config = Eigen::VectorXd::Zero(robot_model_.nq);

    // (2) Locked Legs
    initialize_leg_joints_to_lock();
    for (const auto& joint_name : mixed_joints_to_lock_ids_) {
        if (robot_model_.existJointName(joint_name)) {
            leg_joints.push_back(robot_model_.getJointId(joint_name));
        }
    }
    pinocchio::buildReducedModel(
        robot_model_, 
        geom_robot_model_,
        leg_joints, 
        reference_config, 
        upper_body_, 
        geom_upper_body_
    );
    reference_config = Eigen::VectorXd::Zero(upper_body_.nq);
    std::cout << "Upper body made" <<std::endl;


    // (3) Locked Palms
    initialize_palm_joints_to_lock();
    for (const auto& joint_name : mixed_joints_to_lock_ids_) {
        if (upper_body_.existJointName(joint_name)) {
            palm_joints.push_back(upper_body_.getJointId(joint_name));
        }
    }
    pinocchio::buildReducedModel(
        upper_body_, 
        geom_upper_body_,
        palm_joints, 
        reference_config, 
        upper_body_wo_palms,
        geom_upper_body_wo_palms
    );

    std::cout<< "Models made"<<std::endl;
    
    // Initialize Tools with appropriate models
    ik = new G1_29_ArmIK(
        upper_body_wo_palms, 
        geom_upper_body_wo_palms
    );
    controller = new ImpedanceController(upper_body_wo_palms);
    motion_planner = new VisualServoPlanner();

}

void G1DualArm::initialize_wrist_joints_to_lock() {
    mixed_joints_to_lock_ids_ = {
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint",
        "left_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
        "right_wrist_yaw_joint"
    };
}

void G1DualArm::initialize_leg_joints_to_lock() {
    mixed_joints_to_lock_ids_ = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint", 
        "left_hip_yaw_joint",
        "left_knee_joint", 
        "left_ankle_pitch_joint", 
        "left_ankle_roll_joint",
        "right_hip_pitch_joint", 
        "right_hip_roll_joint", 
        "right_hip_yaw_joint",
        "right_knee_joint", 
        "right_ankle_pitch_joint", 
        "right_ankle_roll_joint",
        "waist_yaw_joint", 
        "waist_roll_joint", 
        "waist_pitch_joint"
    };

    
}

void G1DualArm::initialize_palm_joints_to_lock() {
    mixed_joints_to_lock_ids_ = {
        "left_hand_thumb_0_joint", 
        "left_hand_thumb_1_joint", 
        "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", 
        "left_hand_middle_1_joint",
        "left_hand_index_0_joint", 
        "left_hand_index_1_joint",
        "right_hand_thumb_0_joint", 
        "right_hand_thumb_1_joint", 
        "right_hand_thumb_2_joint",
        "right_hand_index_0_joint", 
        "right_hand_index_1_joint",
        "right_hand_middle_0_joint", 
        "right_hand_middle_1_joint"
    };
    
}

void G1DualArm::add_end_effector_frames() {
    // Add end-effector frames at elbow joints (without wrists)
    pinocchio::JointIndex left_elbow_id, right_elbow_id;
    
    if (robot_config_.NUM_DOF==29){
        left_elbow_id = robot_model_.getJointId("left_wrist_yaw_joint");
        right_elbow_id = robot_model_.getJointId("right_wrist_yaw_joint");
    } else{
        left_elbow_id = robot_model_.getJointId("left_elbow_joint");
        right_elbow_id = robot_model_.getJointId("right_elbow_joint");

    }
    pinocchio::SE3 left_placement(Eigen::Matrix3d::Identity(), 
                                   Eigen::Vector3d(0.01, 0, 0));
    pinocchio::SE3 right_placement(Eigen::Matrix3d::Identity(), 
                                    Eigen::Vector3d(0.01, 0, 0));

    robot_model_.addFrame(pinocchio::Frame("L_ee", left_elbow_id, 
                                             left_placement, pinocchio::OP_FRAME));
    robot_model_.addFrame(pinocchio::Frame("R_ee", right_elbow_id, 
                                              right_placement, pinocchio::OP_FRAME));
}

JointState G1DualArm::grav_ff(JointState current_state){

    std::tie(q,v,e) = jointstate_to_vectors(current_state, robot_model_);

    pinocchio::computeGeneralizedGravity(robot_model_, robot_data_, q);
    grav_torques = robot_data_.g;
    
    return vectors_to_jointstate(q,v,grav_torques, robot_model_);
}

std::tuple<
    Eigen::VectorXd,
    Eigen::VectorXd,
    Eigen::VectorXd
>
jointstate_to_vectors(JointState input, const pinocchio::Model& model){
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(model.nv);

    for (int i = 0; i < input.name.size(); i++){
        auto joint_id = model.getJointId(input.name[i]);
        if (joint_id >= 0 && joint_id < model.njoints){
            auto i_q = model.joints[joint_id].idx_q();
            auto i_v = model.joints[joint_id].idx_v();

            q(i_q) = input.position[i];
            v(i_v) = input.velocity[i];
            e(i_v) = input.effort[i];
        }
    }
    return {q,v,e};
}

JointState vectors_to_jointstate(
    Eigen::VectorXd q, Eigen::VectorXd v, Eigen::VectorXd e,
    const pinocchio::Model& model
){
    JointState result;

    int i_q = 0;
    int i_v = 0;

    for (int joint_id = 1; joint_id < model.njoints; ++joint_id) {
        i_q = model.joints[joint_id].idx_q();
        i_v = model.joints[joint_id].idx_v();

        result.name.push_back(model.names[joint_id]);
        result.position.push_back(q[i_q]);
        result.velocity.push_back(v[i_v]);
        result.effort.push_back(e[i_v]);
    }

    return result;
}

} // ArmPilot namespace