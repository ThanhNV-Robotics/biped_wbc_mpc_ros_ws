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
    biped_pkg    = FindPackageShare("biped_description")
    controllers_pkg = FindPackageShare("legged_controllers")
    this_pkg     = FindPackageShare("test_pinocchio")

    pd_gains_file    = LaunchConfiguration("pd_gains_file").perform(context)
    mjcf_file        = LaunchConfiguration("mjcf_file").perform(context)
    controllers_yaml = PathJoinSubstitution(
        [this_pkg, "config", "controllers.yaml"]
    ).perform(context)

    urdf_file = PathJoinSubstitution([biped_pkg, "urdf", "biped_robot_12dof.urdf"]).perform(context)
    task_file = PathJoinSubstitution([controllers_pkg, "config", "mpc", "task.info"]).perform(context)

    # ── Robot description via xacro ───────────────────────────────────────────
    robot_desc_str = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", urdf_file,
        " use_mjcf_from_topic:=false",
        " use_legged_hw:=false",
        " pd_gains_file:=", pd_gains_file,
        " mjcf_file:=", mjcf_file,
    ]).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=robot_desc_str, value_type=str)
    }

    return [
        # ── Robot state publisher (for TF tree / RViz) ────────────────────────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),

        # ── MuJoCo ros2_control node ──────────────────────────────────────────
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

        # ── Spawn only joint_state_broadcaster + IMU (no MPC controller) ─────
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

        # ── Pinocchio forward kinematics node ─────────────────────────────────
        Node(
            package="test_pinocchio",
            executable="forward_kinematics_node",
            name="forward_kinematics_node",
            output="screen",
            parameters=[{
                "urdf_file":           urdf_file,
                "task_file":           task_file,
                "joint_states_topic":  "/joint_states",
                "base_z":              0.765,
                "mujoco_model_path":   PathJoinSubstitution(
                    [FindPackageShare("biped_description"), "urdf", "mjmodel.xml"]
                ).perform(context),
            }],
        ),
    ]


def generate_launch_description():
    default_pd_gains = PathJoinSubstitution(
        [FindPackageShare("legged_controllers"), "config", "mujoco_interface_pd_gains.yaml"]
    )
    default_mjcf = PathJoinSubstitution(
        [FindPackageShare("biped_description"), "urdf", "mjmodel.xml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "pd_gains_file",
            default_value=default_pd_gains,
            description="PD gains YAML for MuJoCo internal hold-position mode",
        ),
        DeclareLaunchArgument(
            "mjcf_file",
            default_value=default_mjcf,
            description="Path to MJCF model file loaded by MuJoCo",
        ),
        OpaqueFunction(function=launch_setup),
    ])
