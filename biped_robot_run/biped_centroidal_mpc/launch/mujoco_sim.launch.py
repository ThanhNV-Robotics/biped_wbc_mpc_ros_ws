from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *_args, **_kwargs):
    biped_pkg_share = FindPackageShare("biped_description")
    this_pkg_share = FindPackageShare("biped_centroidal_mpc")

    # Resolve the PD gains file path at launch time (overrideable via pd_gains_file:= CLI arg)
    pd_gains_file = LaunchConfiguration("pd_gains_file").perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([biped_pkg_share, "urdf", "biped_robot_12dof.urdf"]),
            " use_pid:=true",
            " headless:=", LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
            " use_udp:=", LaunchConfiguration("use_udp"),
        ]
    )
    robot_description_str = robot_description_content.perform(context)
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    legged_config = PathJoinSubstitution(
        [this_pkg_share, "config", "legged_controller_test.yaml"]
    )

    nodes = []

    # Robot state publisher
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # MuJoCo ros2_control node.
    # pd_gains_file is read by MujocoSystemInterface as a ROS2 parameter on mujoco_ros2_control_node.
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[{"use_sim_time": True, "pd_gains_file": pd_gains_file}],
            on_exit=Shutdown(),
        )
    )

    # # Spawn and activate pd_impedance_controller
    # nodes.append(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["pd_impedance_controller", "--param-file", pd_config],
    #         output="both",
    #     )
    # )

    # # Spawners — PD stack
    # for controller in ["joint_state_broadcaster", "pd_impedance_controller", "imu_sensor_broadcaster"]:
    #     nodes.append(
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=[controller, "--param-file", pd_config],
    #             output="both",
    #         )
    #     )

    # # Spawner — legged controller (plugin under test)
    # nodes.append(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["legged_controller", "--param-file", legged_config],
    #         output="both",
    #     )
    # )

    # # Control actions server
    # nodes.append(
    #     Node(
    #         package="control_actions",
    #         executable="control_actions_server_node",
    #         output="both",
    #         parameters=[{"use_sim_time": True}],
    #     )
    # )

    return nodes


def generate_launch_description():
    default_pd_gains = PathJoinSubstitution(
        [FindPackageShare("biped_centroidal_mpc"), "config", "pd_impedance_controller.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run simulation without visualization window",
        ),
        DeclareLaunchArgument(
            "pd_gains_file",
            default_value=default_pd_gains,
            description="Path to YAML file with PD gains for internal hold-position mode",
        ),
        DeclareLaunchArgument(
            "use_udp",
            default_value="false",
            description="Enable UDP bridge (sends robot state, receives joint commands)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
