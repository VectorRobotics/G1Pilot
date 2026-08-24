#ifndef G1_PILOT_H
#define G1_PILOT_H

#include <Eigen/Dense>
#include <map>

namespace HumanoidPilot {

struct JointState {
    std::vector<std::string> name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};

struct RobotConfig {
    std::string asset_file = "../assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
    std::string asset_root = "../assets/g1/";
    std::string config_file = "../config/g1.yaml";
    int NUM_DOF = 29;
};

class Humanoid {

public:

    Humanoid(
        const RobotConfig* robot_config = nullptr
    );
    ~Humanoid();

    JointState grav_ff(JointState input);

    std::vector<std::string> get_joint_names();

    JointState solve_ik(
        const Eigen::Matrix4d& left_wrist,
        const Eigen::Matrix4d& right_wrist,
        const JointState* current_state = nullptr,
        const Eigen::VectorXd* EE_efrc_L = nullptr,
        const Eigen::VectorXd* EE_efrc_R = nullptr,
        double* l_pos_err = nullptr,
        double* l_rot_err = nullptr,
        double* r_pos_err = nullptr,
        double* r_rot_err = nullptr,
        bool* collision = nullptr
    );

    JointState solve_ik(
        const Eigen::Matrix4d& wrist,
        const bool left = false,
        const JointState* current_state = nullptr,
        const Eigen::VectorXd* EE_efrc = nullptr,
        double* pos_err = nullptr,
        double* rot_err = nullptr,
        bool* collision = nullptr
    );

    bool check_collision(const Eigen::VectorXd& q);

    void reset();

    void update(JointState current_state);

    Eigen::MatrixXd get_current_left_ee_pose();
    Eigen::MatrixXd get_current_right_ee_pose();

    Eigen::VectorXd get_current_left_ee_vel();
    Eigen::VectorXd get_current_right_ee_vel();

    double get_current_left_ee_error();
    double get_current_right_ee_error();

    JointState control_no_arms(
        JointState current_state
    );

    JointState control_both_arms(
        JointState current_state,
        Eigen::Matrix4d desired_l_ee_pose,
        Eigen::Matrix4d desired_r_ee_pose,
        Eigen::VectorXd desired_l_ee_vel = Eigen::VectorXd::Zero(6),
        Eigen::VectorXd desired_r_ee_vel = Eigen::VectorXd::Zero(6)
    );

    JointState control_left_arm(
        JointState current_state,
        Eigen::Matrix4d desired_ee_pose,
        Eigen::VectorXd desired_ee_vel = Eigen::VectorXd::Zero(6)
    );

    JointState control_right_arm(
        JointState current_state,
        Eigen::Matrix4d desired_ee_pose,
        Eigen::VectorXd desired_ee_vel = Eigen::VectorXd::Zero(6)
    );

    std::pair<
    std::vector<Eigen::VectorXd>,
    std::vector<Eigen::MatrixXd>>
    planPath(
        const Eigen::MatrixXd* goal_pose,
        const Eigen::MatrixXd* start_pose = nullptr,
        const Eigen::VectorXd* start_vel = nullptr,
        const Eigen::VectorXd* goal_vel = nullptr,
        const Eigen::VectorXd* start_acc = nullptr,
        const Eigen::VectorXd* goal_acc = nullptr
    );

    std::pair<
    std::vector<Eigen::VectorXd>,
    std::vector<Eigen::MatrixXd>>
    planTrajectory(
        const Eigen::MatrixXd* goal_pose,
        const Eigen::MatrixXd* start_pose = nullptr,
        const Eigen::VectorXd* start_vel = nullptr,
        const Eigen::VectorXd* goal_vel = nullptr,
        const Eigen::VectorXd* start_acc = nullptr,
        const Eigen::VectorXd* goal_acc = nullptr,
        double time_step = 0.1,
        const double MAX_LIN_VEL = 0.1,
        const double MAX_ANG_VEL = 4.0,
        const double MAX_LIN_ACC = 0.05,
        const double MAX_ANG_ACC = 4.0
    );

protected:

    struct Impl;
    std::unique_ptr<Impl> impl_;
    struct EEFrame {
        std::string parent_joint;
        Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    };

    void extract_config();
    void add_end_effector_frames();
    void apply_joint_limit_overrides();

    RobotConfig robot_config_;

    EEFrame left_ee_;
    EEFrame right_ee_;

    std::vector<std::string> locked_joints_;

    struct JointLimit {
        double lower;
        double upper;
    };
    std::map<std::string, JointLimit> joint_limit_overrides_;

};


} // HumanoidPilot namespace


#endif