#include "wbc_controller/wbc_controller.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace biped_controllers
{
    // // config command interface
    // controller_interface::InterfaceConfiguration WBCController::command_interface_configuration() const
    // {
    //     controller_interface::InterfaceConfiguration config;
    //     config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    //     for (const auto &joint : joints_)
    //     {
    //         config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    //         config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    //         config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    //     }
    //     return config;
    // }

    // // config state interface
    // controller_interface::InterfaceConfiguration WBCController::state_interface_configuration() const
    // {
    //     controller_interface::InterfaceConfiguration config;
    //     config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    //     for (const auto &joint : joints_)
    //     {
    //         config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    //     }
    //     if (!enabled_interface_.empty())
    //     {
    //         config.names.push_back(enabled_interface_);
    //     }
    //     return config;
    // }

    controller_interface::CallbackReturn WBCController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        // TODO:
        // process urdf and create tsid robot_wrapper

        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn WBCController::on_activate(const rclcpp_lifecycle::State & previous_state) 
    {
        // claim (state/command interface)
        
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn WBCController::on_deactivate(const rclcpp_lifecycle::State & previous_state) 
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type WBCController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // get reference command from joystick

        // estimate state from hardware interface

        // estimate contact state

        // compute wbc command

        // send command hardware using command interface
        
        
        return controller_interface::return_type::OK;
    }

} // namespace biped_controllers

PLUGINLIB_EXPORT_CLASS(biped_controllers::WBCController, controller_interface::ControllerInterface)
