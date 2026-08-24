#include "core/interfaces.h"
#include <yaml-cpp/yaml.h>

namespace HumanoidPilot
{

void Humanoid::extract_config() {
    YAML::Node root = YAML::LoadFile(robot_config_.config_file);

    std::string active = root["active_config"].as<std::string>();
    YAML::Node config = root["configs"][active];
    if (!config) {
        throw std::runtime_error(
            "[Humanoid] active_config '" + active + "' not found under configs in " + robot_config_.config_file
        );
    }

    YAML::Node categories = root["joint_categories"];

    std::vector<std::string> locked;

    for (const auto& cat_node : config["lock_categories"]) {
        std::string cat_name = cat_node.as<std::string>();
        YAML::Node joints = categories[cat_name];
        if (!joints) {
            throw std::runtime_error(
                "[Humanoid] joint_categories has no entry '" + cat_name + "' (referenced by config '" + active + "')"
            );
        }
        for (const auto& j : joints) {
            locked.push_back(j.as<std::string>());
        }
    }

    for (const auto& j : config["lock_joints"]) {
        locked.push_back(j.as<std::string>());
    }

    std::set<std::string> seen;
    locked_joints_.clear();
    locked_joints_.reserve(locked.size());
    for (const auto& j : locked) {
        if (seen.insert(j).second) {
            locked_joints_.push_back(j);
        }
    }

    auto parse_ee = [](const YAML::Node& n) -> EEFrame {
        EEFrame ee;
        ee.parent_joint = n["parent_joint"].as<std::string>();
        ee.offset = Eigen::Vector3d(
            n["offset"][0].as<double>(),
            n["offset"][1].as<double>(),
            n["offset"][2].as<double>()
        );
        return ee;
    };
    YAML::Node ees = config["end_effectors"];
    left_ee_ = parse_ee(ees["left"]);
    right_ee_ = parse_ee(ees["right"]);

    joint_limit_overrides_.clear();
    YAML::Node overrides = config["joint_limit_overrides"];
    for (const auto& kv : overrides) {
        std::string joint_name = kv.first.as<std::string>();
        JointLimit lim;
        lim.lower = kv.second["lower"].as<double>();
        lim.upper = kv.second["upper"].as<double>();
        joint_limit_overrides_[joint_name] = lim;
    }
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

} // namespace HumanoidPilot