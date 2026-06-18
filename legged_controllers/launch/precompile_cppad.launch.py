"""
Precompile CppAD libraries for the biped MPC controller.

Run this ONCE after changing the robot model (URDF, joint count, contact frames)
or after a clean build. It loads the controller configuration without starting
the MuJoCo simulation, triggering OCS2 to regenerate and compile the CppAD
auto-differentiation libraries.

CppAD compilation typically takes 2-5 minutes. Once done, set
  recompileLibrariesCppAd = false
in config/mpc/task.info to skip recompilation on subsequent launches.

Usage:
    ros2 launch legged_controllers precompile_cppad.launch.py
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, Shutdown, LogInfo
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    biped_share  = get_package_share_directory("biped_description")
    legged_share = get_package_share_directory("legged_controllers")

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("legged_controllers"), "config", "main_controllers.yaml"]
    ).perform(context)

    # Expand the URDF with use_pid:=true so all command interfaces are exported.
    # This must match exactly the URDF OCS2 will parse for its Pinocchio model.
    robot_description_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("biped_description"), "urdf", "biped_robot_12dof.urdf"]),
            " use_pid:=true",
            " headless:=true",
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
            " use_legged_hw:=false",
            " pd_gains_file:=",
            PathJoinSubstitution(
                [FindPackageShare("legged_controllers"), "config", "mujoco_interface_pd_gains.yaml"]
            ),
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    # Controller params — same structure as test_load_mpc_controller.launch.py
    legged_controller_params = {
        "legged_controller": {
            "ros__parameters": {
                "urdf_file":      os.path.join(biped_share, "urdf", "biped_robot_12dof.urdf"),
                "task_file":      os.path.join(legged_share, "config", "mpc", "task.info"),
                "reference_file": os.path.join(legged_share, "config", "mpc", "reference.info"),
            }
        }
    }
    param_file = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="legged_precompile_"
    )
    yaml.dump(legged_controller_params, param_file)
    param_file.close()

    return [
        # robot_state_publisher is required so ros2_control_node can read
        # /robot_description before trying to load the hardware interface.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": False}],
            output="both",
        ),
        # ros2_control_node with MuJoCo backend — starts the controller_manager
        # so the spawner can configure the controller (which triggers CppAD).
        # Shut down the whole launch when this exits.
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                ParameterFile(controllers_yaml, allow_substs=False),
                {"use_sim_time": False},
            ],
            on_exit=Shutdown(),
        ),
        # Spawner configures legged_controller — on_configure() calls
        # setupLeggedInterface() which triggers CppAD code generation.
        # recompileLibrariesCppAd must be true in task.info for this to work.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["legged_controller", "--param-file", param_file.name],
            output="both",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
