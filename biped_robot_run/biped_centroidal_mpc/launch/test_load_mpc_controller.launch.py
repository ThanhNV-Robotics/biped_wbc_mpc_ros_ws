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
    biped_pkg_share = FindPackageShare("biped_description")
    this_pkg_share = FindPackageShare("biped_centroidal_mpc")

    # Robot description — use_pid:=true exports position/velocity/effort interfaces
    # required so the controller_manager can later assign them to legged_controller
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([biped_pkg_share, "urdf", "biped_robot_12dof.urdf"]),
            " use_pid:=true",
            " headless:=", LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
        ]
    )
    robot_description_str = robot_description_content.perform(context)
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    config_file = PathJoinSubstitution(
        [this_pkg_share, "config", "legged_controller_test.yaml"]
    )

    nodes = []

    # # Spawn legged_controller — this is the plugin under test
    # nodes.append(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["legged_controller", "--param-file", config_file],
    #         output="both",
    #     )
    # )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run MuJoCo without a visualization window",
        ),
        OpaqueFunction(function=launch_setup),
    ])
