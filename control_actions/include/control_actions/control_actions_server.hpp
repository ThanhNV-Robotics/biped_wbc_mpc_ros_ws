#pragma once
#include "control_actions/action/move_one_joint.hpp"
#include "control_actions/action/initial_standing.hpp"

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

namespace control_actions
{
    class ControlActionServer : public rclcpp::Node
    {
        public:
            using MoveOneJointAction = control_actions::action::MoveOneJoint;
            using GoalHandleCMoveOneJoint = rclcpp_action::ServerGoalHandle<MoveOneJointAction>;

            using InitialStandingAction = control_actions::action::InitialStanding;
            using GoalHandleInitialStanding = rclcpp_action::ServerGoalHandle<InitialStandingAction>;

            CONTROL_ACTIONS_CPP_PUBLIC
            explicit ControlActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()); //constructor
            void set_mujoco_model (mjModel *mj_model);
            void set_joint_limits (std::unordered_map<std::string, std::pair<double, double>> joint_limits);
            void set_joint_names (std::vector<std::string> joint_names) {joint_names_ = joint_names; return;};        
            std::unordered_map<std::string, double> get_current_joint_positions(std::vector<std::string> joint_names);
            std::vector<std::string> get_joint_names() const { return joint_names_; }

        private:
            // share attributes for all actions
            mjModel *mj_model_; // to read the robot configuration
            std::unordered_map<std::string, double> current_joint_positions_ ; // (joint_name, position)
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
            std::vector<std::string> joint_names_; // list of joint names in the mujoco model
            std::unordered_map<std::string, std::pair<double, double>> joint_limits_; // (joint_name, (lower_limit, upper_limit))

            // for move_one_joint action
            rclcpp_action::Server<MoveOneJointAction>::SharedPtr act_serv_move_one_joint_;
            rclcpp_action::GoalResponse move_one_joint_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveOneJointAction::Goal> goal);
            rclcpp_action::CancelResponse move_one_joint_handle_cancel(const std::shared_ptr<GoalHandleCMoveOneJoint> goal_handle);
            void move_one_joint_handle_accepted(const std::shared_ptr<GoalHandleCMoveOneJoint> goal_handle);
            void move_one_joint_execute (const std::shared_ptr<GoalHandleCMoveOneJoint> goal_handle);

            // for initial_standing action
            rclcpp_action::Server<InitialStandingAction>::SharedPtr act_serv_initial_standing_;
            rclcpp_action::GoalResponse initial_standing_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const InitialStandingAction::Goal> goal);
            rclcpp_action::CancelResponse initial_standing_handle_cancel(const std::shared_ptr<GoalHandleInitialStanding> goal_handle);
            void initial_standing_handle_accepted(const std::shared_ptr<GoalHandleInitialStanding> goal_handle);
            void initial_standing_execute (const std::shared_ptr<GoalHandleInitialStanding> goal_handle);

    };
} // namespace control_actions_cpp
