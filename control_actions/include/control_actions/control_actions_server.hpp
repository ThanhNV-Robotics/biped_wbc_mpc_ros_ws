#include "control_actions/action/move_one_joint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "control_actions/visity_control.h"
#include "mujoco/mujoco.h"
#include <memory>
#include <thread>
#include <unordered_map>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace control_actions_cpp
{
    class ControlActionServer : public rclcpp::Node
    {
    public:
        using ControlActions = control_actions::action::MoveOneJoint;
        using GoalHandleControlActions = rclcpp_action::ServerGoalHandle<ControlActions>;

        CONTROL_ACTIONS_CPP_PUBLIC
        explicit ControlActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        void set_mujoco_model (mjModel *mj_model);
        void set_joint_limits (std::unordered_map<std::string, std::pair<double, double>> joint_limits);
        void set_joint_names (std::vector<std::string> joint_names) {joint_names_ = joint_names; return;};
        
        std::unordered_map<std::string, double> get_current_joint_positions(std::vector<std::string> joint_names);

    private:
        mjModel *mj_model_; // to read the robot configuration
        std::unordered_map<std::string, double> current_joint_positions_ ; // (joint_name, position)
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        std::vector<std::string> joint_names_; // list of joint names in the mujoco model
        std::unordered_map<std::string, std::pair<double, double>> joint_limits_; // (joint_name, (lower_limit, upper_limit))
        rclcpp_action::Server<ControlActions>::SharedPtr act_serv_move_one_joint_;

        rclcpp_action::GoalResponse move_one_joint_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ControlActions::Goal> goal);
        rclcpp_action::CancelResponse move_one_joint_handle_cancel(const std::shared_ptr<GoalHandleControlActions> goal_handle);
        void move_one_joint_handle_accepted(const std::shared_ptr<GoalHandleControlActions> goal_handle);
        void move_one_joint_execute (const std::shared_ptr<GoalHandleControlActions> goal_handle);
        
    };
} // namespace control_actions_cpp
