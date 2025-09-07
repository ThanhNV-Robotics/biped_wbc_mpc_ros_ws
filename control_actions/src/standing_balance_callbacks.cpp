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
        // // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControlActionServer::standing_balance_execute, this, _1), goal_handle}.detach();

        // this returns quickly, so we dont need to make a thread
        // send goal to action InitialStanding.action to init the posture
    }

    void ControlActionServer::standing_balance_execute(const std::shared_ptr<GoalHandleStandingBalance> goal_handle)
    {
        auto initial_standing_client = rclcpp_action::create_client<InitialStandingAction>(this, "/initial_standing");
        bool enable = goal_handle->get_goal()->enable;
        if (enable)
        {
            // Send goal to InitialStanding action to initialize posture
            if (!initial_standing_client->wait_for_action_server(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "InitialStanding action server not available");
                // Handle error, perhaps reject or continue
                return;
            }

            auto goal_msg = InitialStandingAction::Goal();
            goal_msg.hip_height = standing_controller_.get_zc(); // set hip height = controller's zc param
            goal_msg.duration = 1.5;                             // Default duration, adjust as needed

            RCLCPP_INFO(this->get_logger(), "Sending goal to InitialStanding: hip_height=%.2f, duration=%.2f",
                        goal_msg.hip_height, goal_msg.duration);

            auto send_goal_options = rclcpp_action::Client<InitialStandingAction>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](const rclcpp_action::ClientGoalHandle<InitialStandingAction>::SharedPtr &goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by InitialStanding server");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by InitialStanding server");
                }
            };

            send_goal_options.result_callback =
                [this](const rclcpp_action::ClientGoalHandle<InitialStandingAction>::WrappedResult &result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {                    
                    enable_standing_control();
                    RCLCPP_INFO(this->get_logger(), "InitialStanding succeeded, enabling standing control");
                    return;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "InitialStanding failed, not enabling standing control");
                    return;
                }
            };
            initial_standing_client->async_send_goal(goal_msg, send_goal_options);
        }
        else
        {
            disable_standing_control();
            RCLCPP_INFO(this->get_logger(), "Standing control disabled");
            return;
        }
        return;
    }
}