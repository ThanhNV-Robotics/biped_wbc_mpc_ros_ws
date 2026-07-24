"""
Full MuJoCo simulation with MPC locomotion controller and remote control GUI.

Launches:
  1. robot_state_publisher    — publishes /robot_description
  2. mujoco_ros2_control node — MuJoCo physics + controller_manager
  3. joint_state_broadcaster + imu_sensor_broadcaster
  4. legged_controller        — MPC + state estimator
  5. base_velocity_controller_gui (remote_control) — GUI for pelvis height and
     base velocity commands, publishes to /humanoid/walking_velocity_command
  6. robot_state_visualization — second MuJoCo window showing the estimated
     state from /legged_robot_mpc_observation

Usage:
    ros2 launch legged_controllers mpc_mujoco_sim.launch.py
    ros2 launch legged_controllers mpc_mujoco_sim.launch.py pd_gains_file:=/path/to/gains.yaml
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    biped_share  = get_package_share_directory("biped_description")
    legged_share = get_package_share_directory("legged_controllers")

    pd_gains_file = LaunchConfiguration("pd_gains_file").perform(context)
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("legged_controllers"), "config", "main_controllers.yaml"]
    ).perform(context)

    robot_description_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("biped_description"), "urdf", "biped_robot_12dof.urdf"]),
            " use_mjcf_from_topic:=false",
            " use_legged_hw:=false",
            " pd_gains_file:=",     pd_gains_file,
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    legged_controller_params = {
        "legged_controller": {
            "ros__parameters": {
                "urdf_file":      os.path.join(biped_share,  "urdf",   "biped_robot_12dof.urdf"),
                "task_file":      os.path.join(legged_share, "config", "mpc", "task.info"),
                "reference_file": os.path.join(legged_share, "config", "mpc", "reference.info"),
            }
        }
    }
    legged_ctrl_param_file = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="legged_ctrl_"
    )
    yaml.dump(legged_controller_params, legged_ctrl_param_file)
    legged_ctrl_param_file.close()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": True}],
            output="both",
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                ParameterFile(controllers_yaml, allow_substs=False),
                {"use_sim_time": True},
            ],
            on_exit=Shutdown(),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "imu_sensor_broadcaster",
                "--param-file", controllers_yaml,
            ],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["legged_controller", "--param-file", legged_ctrl_param_file.name],
            output="both",
        ),
        Node(
            package="remote_control",
            executable="base_velocity_controller_gui",
            name="base_velocity_controller_gui",
            output="screen",
        ),
        Node(
            package="robot_state_visualization",
            executable="robot_state_visualization_node",
            name="state_estimation_visualizer",
            output="screen",
            parameters=[{
                "mujoco_model_path": os.path.join(biped_share, "mjcf", "biped_robot_12dof.xml"),
            }],
        ),
    ]


def generate_launch_description():
    default_pd_gains = PathJoinSubstitution(
        [FindPackageShare("legged_controllers"), "config", "mujoco_interface_pd_gains.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "pd_gains_file",
            default_value=default_pd_gains,
            description="Path to YAML file with PD gains for the MuJoCo hardware interface",
        ),
        OpaqueFunction(function=launch_setup),
    ])
