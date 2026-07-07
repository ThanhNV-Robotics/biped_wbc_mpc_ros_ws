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

# Test launch for legged_controller lifecycle verification.
#
# Steps:
#   1. Build robot description — xacro expands the URDF with MuJoCo hardware tags.
#   2. robot_state_publisher — publishes /robot_description topic (required by
#      ros2_control_node to initialise the hardware interface).
#   3. mujoco_ros2_control node — starts the controller_manager with MuJoCo as
#      the hardware backend.
#   4. Broadcaster spawner — loads joint_state_broadcaster and imu_sensor_broadcaster
#      so joint/IMU state interfaces are available to other controllers.
#   5. legged_controller spawner — loads and activates LeggedController, which
#      triggers on_init() → on_configure() → on_activate() in sequence.


def launch_setup(context):
    # ── Step 1: Resolve package paths and build robot description ────────────
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
            " pd_gains_file:=",      pd_gains_file,
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    # legged_controller requires 3 file paths in on_init(). Paths depend on the
    # install prefix so they cannot be hardcoded in a static YAML file.
    #
    # launch_ros always wraps Python dict params under /**:ros__parameters: with
    # dot-flattened keys — the controller node cannot match them as its own params.
    # Writing the YAML ourselves gives the exact nested structure the node expects:
    #   legged_controller:
    #     ros__parameters:
    #       urdf_file: ...
    # Passing this file via --param-file on the spawner delivers it directly to
    # the controller's lifecycle node at load time.
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
        # ── Step 2: Publish robot description topic ───────────────────────────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": True}],
            output="both",
        ),
        # ── Step 3: Start MuJoCo + controller_manager ─────────────────────────
        # Shuts down the whole launch if the simulator exits (e.g. window closed).
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                ParameterFile(controllers_yaml, allow_substs=False),
                {"use_sim_time": True},  # must be last — overrides any stale value
            ],
            on_exit=Shutdown(),
        ),
        # ── Step 4: Spawn state broadcasters ─────────────────────────────────
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
        # ── Step 5: Spawn legged_controller (controller under test) ───────────
        # on_init() reads urdf_file/task_file/reference_file and returns SUCCESS.
        # on_configure() and on_activate() are stubs until step 3 is implemented.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["legged_controller", "--param-file", legged_ctrl_param_file.name],
            output="both",
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
            description="Path to YAML file with PD gains for MuJoCo hold-position mode",
        ),
        OpaqueFunction(function=launch_setup),
    ])
