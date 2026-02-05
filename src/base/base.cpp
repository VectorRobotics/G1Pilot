#include "base/base.h"

#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "arm_control/impedance_control.h"
#include "arm_ik/robot_arm_ik.h"
#include "arm_mp/poly_traj_gen.h"

namespace ArmPilot {

G1DualArm::G1DualArm(
    const RobotConfig* robot_config
)
{
    std::cout << "Init Started" <<std::endl;
    robot_config_ = RobotConfig();
    if (robot_config != nullptr) {
        robot_config_ = *robot_config;
    }

    // (1) Basic Model
    pinocchio::urdf::buildModel(robot_config_.asset_file, robot_model_);
    std::cout << "Frame model made" <<std::endl;
    pinocchio::urdf::buildGeom(
        robot_model_, 
        robot_config_.asset_file, 
        pinocchio::COLLISION, 
        geom_robot_model_, 
        robot_config_.asset_root
    );
    std::cout << "Geom model made" <<std::endl;
    add_end_effector_frames();
    reference_config = Eigen::VectorXd::Zero(robot_model_.nq);
    std::cout << "Full model made" <<std::endl;


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
    motion_planner = new PolynomialTrajectoryGenerator( );

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

} // ArmPilot namespace