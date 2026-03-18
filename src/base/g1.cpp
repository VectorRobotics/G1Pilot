#include "base/g1.h"

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
    categorize_joints();

    pinocchio::urdf::buildModel(robot_config_.asset_file, model_g1_29);
    pinocchio::urdf::buildGeom(
        model_g1_29, 
        robot_config_.asset_file, 
        pinocchio::COLLISION, 
        geom_g1_29, 
        robot_config_.asset_root
    );
    add_end_effector_frames();

    reduce_model(
        waist_joints,
        model_g1_29,
        geom_g1_29,
        model_g1_26,
        geom_g1_26
    );

    reduce_model(
        leg_joints,
        model_g1_26,
        geom_g1_26,
        model_g1_14,
        geom_g1_14
    );

    reduce_model(
        hand_joints,
        model_g1_14,
        geom_g1_14,
        model_g1_14_wo_hands,
        geom_g1_14_wo_hands
    );

    data_g1_26 = pinocchio::Data(model_g1_26);

    // Initialize Tools with appropriate models
    ik = new HumanoidIK(
        model_g1_14_wo_hands, 
        geom_g1_14_wo_hands
    );
    controller = new IKTrajTracker(model_g1_14_wo_hands, ik);
    motion_planner = new VisualServoPlanner();

}

void G1DualArm::categorize_joints() {

    left_leg_joints = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint", 
        "left_hip_yaw_joint",
        "left_knee_joint", 
        "left_ankle_pitch_joint", 
        "left_ankle_roll_joint"
    };

    right_leg_joints = {
        "right_hip_pitch_joint", 
        "right_hip_roll_joint", 
        "right_hip_yaw_joint",
        "right_knee_joint", 
        "right_ankle_pitch_joint", 
        "right_ankle_roll_joint"
    };

    waist_joints = {
        // "waist_yaw_joint", 
        "waist_roll_joint", 
        "waist_pitch_joint"
    };

    left_upper_arm_joints = {
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint"
    };

    right_upper_arm_joints = {
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint"
    };

    left_wrist_joints = {
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint",
        "left_wrist_yaw_joint"
    };

    right_wrist_joints = {
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
        "right_wrist_yaw_joint"
    };

    left_hand_joints = {
        "left_hand_thumb_0_joint", 
        "left_hand_thumb_1_joint", 
        "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", 
        "left_hand_middle_1_joint",
        "left_hand_index_0_joint", 
        "left_hand_index_1_joint"
    };

    right_hand_joints = {
        "right_hand_thumb_0_joint", 
        "right_hand_thumb_1_joint", 
        "right_hand_thumb_2_joint",
        "right_hand_middle_0_joint", 
        "right_hand_middle_1_joint",
        "right_hand_index_0_joint", 
        "right_hand_index_1_joint"
    };

    leg_joints.reserve(
        left_leg_joints.size() + 
        right_leg_joints.size()
    );

    leg_joints.insert(leg_joints.end(), left_leg_joints.begin(), left_leg_joints.end());
    leg_joints.insert(leg_joints.end(), right_leg_joints.begin(), right_leg_joints.end());
    
    upper_arm_joints.reserve(
        left_upper_arm_joints.size() + 
        right_upper_arm_joints.size()
    );

    upper_arm_joints.insert(upper_arm_joints.end(), left_upper_arm_joints.begin(), left_upper_arm_joints.end());
    upper_arm_joints.insert(upper_arm_joints.end(), right_upper_arm_joints.begin(), right_upper_arm_joints.end());

    wrist_joints.reserve(
        left_wrist_joints.size() + 
        right_wrist_joints.size()
    );

    wrist_joints.insert(wrist_joints.end(), left_wrist_joints.begin(), left_wrist_joints.end());
    wrist_joints.insert(wrist_joints.end(), right_wrist_joints.begin(), right_wrist_joints.end());

    left_arm_joints.reserve(
        left_upper_arm_joints.size() + 
        left_wrist_joints.size()
    );

    left_arm_joints.insert(left_arm_joints.end(), left_upper_arm_joints.begin(), left_upper_arm_joints.end());
    left_arm_joints.insert(left_arm_joints.end(), left_wrist_joints.begin(), left_wrist_joints.end());
    
    right_arm_joints.reserve(
        right_upper_arm_joints.size() + 
        right_wrist_joints.size()
    );

    right_arm_joints.insert(right_arm_joints.end(), right_upper_arm_joints.begin(), right_upper_arm_joints.end());
    right_arm_joints.insert(right_arm_joints.end(), right_wrist_joints.begin(), right_wrist_joints.end());
    
    hand_joints.reserve(
        left_hand_joints.size() + 
        right_hand_joints.size()
    );

    hand_joints.insert(hand_joints.end(), left_hand_joints.begin(), left_hand_joints.end());
    hand_joints.insert(hand_joints.end(), right_hand_joints.begin(), right_hand_joints.end());

}

void G1DualArm::add_end_effector_frames() {
    // Add end-effector frames at elbow joints (without wrists)
    pinocchio::JointIndex left_elbow_id, right_elbow_id;
    
    if (robot_config_.NUM_DOF==29){
        left_elbow_id = model_g1_29.getJointId("left_wrist_yaw_joint");
        right_elbow_id = model_g1_29.getJointId("right_wrist_yaw_joint");
    } else{
        left_elbow_id = model_g1_29.getJointId("left_elbow_joint");
        right_elbow_id = model_g1_29.getJointId("right_elbow_joint");

    }
    pinocchio::SE3 left_placement(Eigen::Matrix3d::Identity(), 
                                   Eigen::Vector3d(
                                    0.0415+0.0777+0.0458+0.0458, 
                                    0.003+0.0016, 
                                    0.0+0.0285
                                ));
    pinocchio::SE3 right_placement(Eigen::Matrix3d::Identity(), 
                                    Eigen::Vector3d(
                                    0.0415+0.0777+0.0458+0.0458, 
                                    -0.003-0.0016, 
                                    0.0+0.0285
                                ));

    model_g1_29.addFrame(pinocchio::Frame("L_ee", left_elbow_id, 
                                             left_placement, pinocchio::OP_FRAME));
    model_g1_29.addFrame(pinocchio::Frame("R_ee", right_elbow_id, 
                                              right_placement, pinocchio::OP_FRAME));
}

JointState G1DualArm::grav_ff(JointState current_state){

    std::tie(q,v,e) = jointstate_to_vectors(current_state, model_g1_26);

    pinocchio::computeGeneralizedGravity(model_g1_26, data_g1_26, q);
    // grav_torques = 1.052*data_g1_26.g;
    grav_torques = data_g1_26.g;
    
    return vectors_to_jointstate(q,v,grav_torques, model_g1_26);
}

void G1DualArm::reduce_model(
    std::vector<std::string> joint_names,
    pinocchio::Model &model_in,
    pinocchio::GeometryModel &gmodel_in,
    pinocchio::Model &model_out,
    pinocchio::GeometryModel &gmodel_out
){
    Eigen::VectorXd reference_config = Eigen::VectorXd::Zero(model_in.nq);
    std::vector<pinocchio::JointIndex> joints_idx;

    for (const auto& joint_name : joint_names) {
        if (model_in.existJointName(joint_name)) {
            joints_idx.push_back(
                model_in.getJointId(joint_name)
            );
        }
    }

    pinocchio::buildReducedModel(
        model_in, 
        gmodel_in,
        joints_idx, 
        reference_config, 
        model_out,
        gmodel_out
    );
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