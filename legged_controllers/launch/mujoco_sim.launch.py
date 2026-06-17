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


def launch_setup(context, *_args, **_kwargs):
    biped_pkg_share  = FindPackageShare("biped_description")
    this_pkg_share   = FindPackageShare("legged_controllers")

    pd_gains_file    = LaunchConfiguration("pd_gains_file").perform(context)
    headless         = LaunchConfiguration("headless").perform(context)
    controllers_yaml = PathJoinSubstitution(
        [this_pkg_share, "config", "main_controllers.yaml"]
    ).perform(context)

    # ── MuJoCo robot description (MujocoSystemInterface plugin) ──────────────
    mujoco_robot_desc_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([biped_pkg_share, "urdf", "biped_robot_12dof.urdf"]),
            " use_pid:=true",
            " headless:=",          headless,
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
            " use_legged_hw:=false",
            " pd_gains_file:=",     pd_gains_file,
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=mujoco_robot_desc_str, value_type=str)
    }

    return [
        # ── Robot state publisher ─────────────────────────────────────────────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        # ── MuJoCo ros2_control node ──────────────────────────────────────────
        # robot_description comes from /robot_description topic.
        # pd_gains_file is a URDF hardware parameter — passed via xacro above.
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
        # ── Controller spawner ────────────────────────────────────────────────
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
    ]


def generate_launch_description():
    default_pd_gains = PathJoinSubstitution(
        [FindPackageShare("legged_controllers"), "config", "mujoco_interface_pd_gains.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run MuJoCo without a GUI window",
        ),
        DeclareLaunchArgument(
            "pd_gains_file",
            default_value=default_pd_gains,
            description="Path to YAML file with PD gains for MuJoCo internal hold-position mode",
        ),
        OpaqueFunction(function=launch_setup),
    ])
