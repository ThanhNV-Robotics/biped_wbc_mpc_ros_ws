#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    #Get the YAML config file path
    standing_balance_controller_cfg_file = PathJoinSubstitution(
        [
            FindPackageShare("control_actions"),
            "config",
            "StandingBalanceController.yaml",
        ]
    )

    # r6bot_controller_cfg_file = PathJoinSubstitution(
    #         [
    #             FindPackageShare("control_actions"),
    #             "config",
    #             "r6bot_controller.yaml",
    #         ]
    #     )

    # r6bot_controller_node = Node(
    #         package="controller_manager",
    #         executable="ros2_control_node",
    #         parameters=[r6bot_controller_cfg_file],
    #         output="screen"
    #     )
        

    standing_balance_controller_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[standing_balance_controller_cfg_file],
            output="screen"
        )
    # Simple spawner node to load the controller
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "standing_balance_controller", 
            "--param-file", 
            standing_balance_controller_cfg_file
        ],
        output="screen"
    )

    # r6bot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "r6bot_controller", 
    #         "--param-file", 
    #         r6bot_controller_cfg_file
    #     ],
    #     output="screen"
    # )
    
    return LaunchDescription([
        standing_balance_controller_node,
        controller_spawner
        # TimerAction(period=3.0, actions=[controller_spawner])
        # r6bot_controller_node,
        # r6bot_controller_spawner
    ])