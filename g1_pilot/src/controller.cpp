#include <g1_pilot/g1_pilot.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "helper_funcs.h"

#include "humanoid_manipulation_interfaces/action/control_trajectory.hpp"

#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

using namespace HumanoidPilot;
using namespace std::chrono_literals;

using TrajectoryControllerAction = humanoid_manipulation_interfaces::action::ControlTrajectory;
using GoalHandleControl = rclcpp_action::ServerGoalHandle<TrajectoryControllerAction>;

class TrajectoryControllerActionServer : public rclcpp::Node
{
public:
    ~TrajectoryControllerActionServer()
    {
        shutting_down_.store(true);
        goal_cv_.notify_all();
        if (execute_thread_.joinable()) {
            execute_thread_.join();
        }
    }

    TrajectoryControllerActionServer() : Node("trajectory_controller_action_server")
    {
        /* Initialize arm handle */
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("g1_description");
        std::string g1_pilot_share_directory = ament_index_cpp::get_package_share_directory("g1_pilot");

        std::string default_asset_file = package_share_directory + "/assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
        std::string default_asset_root = package_share_directory + "/assets/g1/";
        std::string default_config_file = g1_pilot_share_directory + "/config/g1.yaml";
        int default_n_dof = 29;

        this->declare_parameter<std::string>("asset_file", default_asset_file);
        this->declare_parameter<std::string>("asset_root", default_asset_root);
        this->declare_parameter<std::string>("config_file", default_config_file);
        this->declare_parameter<int>("num_dof", default_n_dof);

        RobotConfig config;
        config.asset_file = this->get_parameter("asset_file").as_string();
        config.asset_root = this->get_parameter("asset_root").as_string();
        config.config_file = this->get_parameter("config_file").as_string();
        config.NUM_DOF = this->get_parameter("num_dof").as_int();

        arm_handle_ = std::make_unique<Humanoid>(&config);

        /* Publishers */
        this->declare_parameter<std::string>("position_control_topic", "position_control");
        this->declare_parameter<std::string>("left_ee_pose_topic", "left_ee_pose");
        this->declare_parameter<std::string>("right_ee_pose_topic", "right_ee_pose");

        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            this->get_parameter("position_control_topic").as_string(), 10);

        left_ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("left_ee_pose_topic").as_string(), 10);

        right_ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("right_ee_pose_topic").as_string(), 10);

        /* Subscribers */
        this->declare_parameter<double>("waypoint_error_margin", 0.007);
        this->declare_parameter<std::string>("feedback_topic", "feedback");

        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            this->get_parameter("feedback_topic").as_string(), 10,
            std::bind(&TrajectoryControllerActionServer::handle_new_feedback_, this, std::placeholders::_1));

        /* Action server */
        action_server_ = rclcpp_action::create_server<TrajectoryControllerAction>(
            this,
            "trajectory_controller",
            std::bind(&TrajectoryControllerActionServer::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryControllerActionServer::handle_cancel_, this, std::placeholders::_1),
            std::bind(&TrajectoryControllerActionServer::handle_accepted_, this, std::placeholders::_1));

        /* Control loop timer */
        control_loop_ = this->create_wall_timer(50ms, std::bind(&TrajectoryControllerActionServer::controller_, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory controller action server ready");
    }

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_ee_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_ee_pose_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_loop_;

    // Action server
    rclcpp_action::Server<TrajectoryControllerAction>::SharedPtr action_server_;

    // Arm Handle
    std::unique_ptr<Humanoid> arm_handle_;

    // EE state
    Eigen::MatrixXd left_ee_pose_;
    Eigen::MatrixXd right_ee_pose_;

    // Trajectory and control state
    std::vector<Eigen::MatrixXd> trajectory_;
    double error_ = 0.0;
    JointState cmd_;
    JointState current_state_;

    // Feedback state mutex (protects current_state_)
    std::mutex feedback_mutex_;
    std::mutex goal_mutex_;

    // Active goal tracking
    std::shared_ptr<GoalHandleControl> active_goal_handle_;
    bool left_arm_active = false;
    uint32_t active_total_waypoints_ = 0;
    std::condition_variable goal_cv_;

    // Thread management
    std::thread execute_thread_;
    std::atomic<bool> shutting_down_{false};

    // --- Action server callbacks ---
    rclcpp_action::GoalResponse handle_goal_(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const TrajectoryControllerAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received control request for %s arm",
                    goal->left_arm ? "left" : "right");

        if (!goal->trajectory.poses.empty()) {
            Eigen::MatrixXd goal_matrix = create_se3(
                goal->trajectory.poses.back().pose.orientation.w,
                goal->trajectory.poses.back().pose.orientation.x,
                goal->trajectory.poses.back().pose.orientation.y,
                goal->trajectory.poses.back().pose.orientation.z,
                goal->trajectory.poses.back().pose.position.x,
                goal->trajectory.poses.back().pose.position.y,
                goal->trajectory.poses.back().pose.position.z
            );

            std::string limit_reason;
            if (!isWithinLimits(goal_matrix, limit_reason)){
                RCLCPP_WARN(this->get_logger(),
                    "Goal out of bounds, rejecting: %s", limit_reason.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            trajectory_ = convertToTrajectory(goal->trajectory, true);
            active_total_waypoints_ = trajectory_.size();
            left_arm_active = goal->left_arm;

            if (active_goal_handle_ && active_goal_handle_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "Preempting active goal");
                auto result = std::make_shared<TrajectoryControllerAction::Result>();
                result->success = false;
                result->final_error = -2.0;
                active_goal_handle_->abort(result);
                active_goal_handle_ = nullptr;
            }
        }
        goal_cv_.notify_all();

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_(
        const std::shared_ptr<GoalHandleControl> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_success_(const std::shared_ptr<GoalHandleControl> goal_handle){
        auto result = std::make_shared<TrajectoryControllerAction::Result>();
        result->success = true;
        result->final_error = error_;
        goal_handle->succeed(result);
        active_goal_handle_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "Goal succeeded with error: %f", result->final_error);
        arm_handle_->ik->reset();
        return;
    }

    void handle_accepted_(const std::shared_ptr<GoalHandleControl> goal_handle)
    {
        // Join previous thread before starting a new one
        if (execute_thread_.joinable()) {
            execute_thread_.join();
        }
        execute_thread_ = std::thread(
            &TrajectoryControllerActionServer::execute_goal_, this, goal_handle);
    }

    void execute_goal_(const std::shared_ptr<GoalHandleControl> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        bool left = goal->left_arm;

        RCLCPP_INFO(this->get_logger(), "Executing goal for %s arm", left ? "left" : "right");

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            active_goal_handle_ = goal_handle;
        }

        while (rclcpp::ok() && !shutting_down_.load()) {

            {
                std::unique_lock<std::mutex> lock(goal_mutex_);

                if (active_goal_handle_ != goal_handle) {
                    RCLCPP_INFO(this->get_logger(), "Goal preempted, exiting execute thread");
                    return;
                }

                if (goal_handle->is_canceling()) {
                    trajectory_.clear();
                    arm_handle_->ik->reset();
                    auto result = std::make_shared<TrajectoryControllerAction::Result>();
                    result->success = false;
                    result->final_error = -1.0;
                    goal_handle->canceled(result);
                    active_goal_handle_ = nullptr;
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                if (trajectory_.size() <= 1) {
                    trajectory_.clear();
                    arm_handle_->ik->reset();
                    handle_success_(goal_handle);
                }

                // Publish feedback
                auto feedback = std::make_shared<TrajectoryControllerAction::Feedback>();
                feedback->current_error = error_;
                feedback->waypoints_remaining = trajectory_.size();
                feedback->total_waypoints = active_total_waypoints_;
                goal_handle->publish_feedback(feedback);

                // Wait with timeout instead of busy-polling
                goal_cv_.wait_for(lock, 100ms);
            }
        }
    }

    void controller_()
    {
        std::scoped_lock lock(feedback_mutex_, goal_mutex_);

        if (current_state_.name.empty()) {
            return;
        }

        arm_handle_->controller->update(current_state_);

        left_ee_pose_ = arm_handle_->controller->get_current_left_ee_pose();
        right_ee_pose_ = arm_handle_->controller->get_current_right_ee_pose();

        left_ee_pose_pub_->publish(convertToPoseStamped(left_ee_pose_, "pelvis", this->get_clock()->now()));
        right_ee_pose_pub_->publish(convertToPoseStamped(right_ee_pose_, "pelvis", this->get_clock()->now()));


        if (trajectory_.empty()){
            return;
        }

        if (left_arm_active){
            cmd_ = arm_handle_->controller->control_left_arm(
                current_state_,
                trajectory_.back()
            );
        }
        else {
            cmd_ = arm_handle_->controller->control_right_arm(
                current_state_,
                trajectory_.back()
            );
        }

        if (trajectory_.size() > 0){
            error_ = left_arm_active ?
                            arm_handle_->controller->get_current_left_ee_error() :
                            arm_handle_->controller->get_current_right_ee_error();

            if (error_ < this->get_parameter("waypoint_error_margin").as_double()){
                trajectory_.pop_back();
                goal_cv_.notify_all();
            }
            RCLCPP_INFO(this->get_logger(), "Error: %f, waypoints remaining: %lu", error_, trajectory_.size());
        }

        auto cmd_msg = sensor_msgs::msg::JointState();
        cmd_msg.header.stamp = this->get_clock()->now();

        cmd_msg.name = cmd_.name;
        cmd_msg.position = cmd_.position;
        cmd_msg.velocity = cmd_.velocity;
        cmd_msg.effort = cmd_.effort;

        joint_states_pub_->publish(cmd_msg);

    }

    void handle_new_feedback_(sensor_msgs::msg::JointState::UniquePtr msg)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        current_state_.name = msg->name;
        current_state_.position = msg->position;
        current_state_.velocity = msg->velocity;
        current_state_.effort = msg->effort;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryControllerActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
