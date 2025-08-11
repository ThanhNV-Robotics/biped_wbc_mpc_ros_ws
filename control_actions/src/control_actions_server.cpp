#include "functional"
#include <memory>
#include <thread>

#include "control_actions/action/move_one_joint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "control_actions/visity_control.h"

namespace control_actions_cpp
{
    class ControlActionServer : public rclcpp::Node
    {
    public:
        using ControlActions = control_actions::action::MoveOneJoint;
        using GoalHandleControlActions = rclcpp_action::ServerGoalHandle<ControlActions>;

        // constructor
        CONTROL_ACTIONS_CPP_PUBLIC // this macro is for visibilty contro
            explicit ControlActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("control_action_server", options)
        {
            using namespace std::placeholders;
            this->action_server_ = rclcpp_action::create_server<ControlActions>(
                this,
                "move_one_joint",
                std::bind(&ControlActionServer::handle_goal, this, _1, _2),
                std::bind(&ControlActionServer::handle_cancel, this, _1),
                std::bind(&ControlActionServer::handle_accepted, this, _1));
        }

    private:
        rclcpp_action::Server<ControlActions>::SharedPtr action_server_;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ControlActions::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with: %s , target position: %f", goal->joint_name.c_str(), goal->target_position);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleControlActions> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void handle_accepted(const std::shared_ptr<GoalHandleControlActions> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&ControlActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute (const std::shared_ptr<GoalHandleControlActions> goal_handle)
        {
            
        }
    };
}
