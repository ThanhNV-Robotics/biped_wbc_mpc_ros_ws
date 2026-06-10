from launch import LaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *_args, **_kwargs):
    biped_pkg_share = FindPackageShare("biped_description")
    this_pkg_share  = FindPackageShare("biped_centroidal_mpc")

    robot_desc_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([biped_pkg_share, "urdf", "biped_robot_12dof.urdf"]),
            " use_legged_hw:=true",
            " legged_hw_use_sim:=false",
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=robot_desc_str, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [this_pkg_share, "config", "legged_hw_controllers.yaml"]
    ).perform(context)

    return [
        # Root-level RSP — joint_state_broadcaster in Jazzy subscribes to the
        # absolute /robot_description topic regardless of its own namespace.
        # Without this it waits indefinitely and triggers the 5-second timeout.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": False}],
        ),

        # Namespaced RSP — controller_manager looks for robot_description
        # relative to its own /legged_hw namespace.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="legged_hw",
            output="both",
            parameters=[robot_description, {"use_sim_time": False}],
        ),

        # legged_hw controller_manager — reads UDP state from STM32
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="legged_hw",
            output="both",
            parameters=[
                robot_description,
                ParameterFile(controllers_yaml, allow_substs=False),
                {"use_sim_time": False},  # overrides use_sim_time in the YAML
            ],
            on_exit=Shutdown(),
        ),

        # Single spawner activates both controllers atomically — avoids the
        # switch_controller timeout that happens when two spawners race.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "imu_sensor_broadcaster",
                "--controller-manager", "/legged_hw/controller_manager",
                "--param-file", controllers_yaml,
            ],
            output="both",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
