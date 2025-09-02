#include <control_actions/control_actions_server.hpp>

namespace control_actions
{
    // for standing_balance action
    rclcpp_action::GoalResponse ControlActionServer::standing_balance_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const StandingBalanceAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for standing_balance");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControlActionServer::standing_balance_handle_cancel(const std::shared_ptr<GoalHandleStandingBalance> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request for standing_balance");
         (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ControlActionServer::standing_balance_handle_accepted(const std::shared_ptr<GoalHandleStandingBalance> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControlActionServer::standing_balance_execute, this, _1), goal_handle}.detach();
    }

    void ControlActionServer::standing_balance_execute(const std::shared_ptr<GoalHandleStandingBalance> goal_handle)
    {
        return;
    }

}