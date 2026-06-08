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
    this_pkg_share  = FindPackageShare("biped_centroidal_mpc")

    pd_gains_file  = LaunchConfiguration("pd_gains_file").perform(context)
    use_legged_hw  = LaunchConfiguration("use_legged_hw").perform(context) == "true"

    # When legged_hw is active the sim must broadcast state via UDP.
    udp_flag = "true" if use_legged_hw else LaunchConfiguration("use_udp").perform(context)

    # ── MuJoCo robot description (MujocoSystemInterface plugin) ──────────────
    mujoco_robot_desc_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([biped_pkg_share, "urdf", "biped_robot_12dof.urdf"]),
            " use_pid:=true",
            " headless:=",         LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
            " use_udp:=",          udp_flag,
            " use_legged_hw:=false",
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=mujoco_robot_desc_str, value_type=str)
    }

    nodes = []

    # ── Robot state publisher ─────────────────────────────────────────────────
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # ── MuJoCo ros2_control node ──────────────────────────────────────────────
    # pd_gains_file is read by MujocoSystemInterface as a ROS2 parameter.
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

    # ── legged_hw: separate ros2_control node that bridges UDP → ros2_control ─
    if use_legged_hw:
        legged_hw_controllers = PathJoinSubstitution(
            [this_pkg_share, "config", "legged_hw_controllers.yaml"]
        ).perform(context)

        legged_hw_robot_desc_str = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([biped_pkg_share, "urdf", "biped_robot_12dof.urdf"]),
                " use_legged_hw:=true",
                " legged_hw_sim_ip:=",     LaunchConfiguration("legged_hw_sim_ip"),
                " legged_hw_state_port:=", LaunchConfiguration("legged_hw_state_port"),
                " legged_hw_cmd_port:=",   LaunchConfiguration("legged_hw_cmd_port"),
            ]
        ).perform(context)

        legged_hw_robot_description = {
            "robot_description": ParameterValue(
                value=legged_hw_robot_desc_str, value_type=str
            )
        }

        # Jazzy controller_manager waits for robot_description on the topic, not
        # just the parameter.  A namespaced robot_state_publisher publishes to
        # /legged_hw/robot_description which the namespaced controller_manager finds.
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="legged_hw",
                output="both",
                parameters=[legged_hw_robot_description, {"use_sim_time": True}],
            )
        )

        # Standard controller_manager node — legged_hw namespace avoids conflicts
        # with the MuJoCo controller_manager.
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="legged_hw",
                output="both",
                parameters=[
                    legged_hw_robot_description,
                    {"use_sim_time": True},
                    ParameterFile(legged_hw_controllers, allow_substs=False),
                ],
                on_exit=Shutdown(),
            )
        )

        # Spawners target /legged_hw/controller_manager.
        # --param-file delivers per-controller config (sensor_name, joints, etc.)
        for controller in ["joint_state_broadcaster", "imu_sensor_broadcaster"]:
            nodes.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        controller,
                        "--controller-manager", "/legged_hw/controller_manager",
                        "--param-file", legged_hw_controllers,
                    ],
                    output="both",
                )
            )

    return nodes


def generate_launch_description():
    default_pd_gains = PathJoinSubstitution(
        [FindPackageShare("biped_centroidal_mpc"), "config", "pd_impedance_controller.yaml"]
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
        DeclareLaunchArgument(
            "use_udp",
            default_value="false",
            description="Enable MuJoCo UDP state bridge (auto-enabled when use_legged_hw:=true)",
        ),
        DeclareLaunchArgument(
            "use_legged_hw",
            default_value="false",
            description="Start legged_hw ros2_control node alongside MuJoCo (enables UDP automatically)",
        ),
        DeclareLaunchArgument(
            "legged_hw_sim_ip",
            default_value="127.0.0.1",
            description="IP of the machine running MuJoCo (legged_hw sends commands here)",
        ),
        DeclareLaunchArgument(
            "legged_hw_state_port",
            default_value="9870",
            description="UDP port legged_hw listens on for state packets from MuJoCo",
        ),
        DeclareLaunchArgument(
            "legged_hw_cmd_port",
            default_value="9871",
            description="UDP port legged_hw sends command packets to MuJoCo",
        ),
        OpaqueFunction(function=launch_setup),
    ])
