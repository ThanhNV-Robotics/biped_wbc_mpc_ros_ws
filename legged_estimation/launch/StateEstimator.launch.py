from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ────────────────────────────────────────────────────
    task_file_arg = DeclareLaunchArgument(
        "task_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("biped_centroidal_mpc"), "config", "mpc", "task.info"]
        ),
        description="Absolute path to the OCS2 task.info file",
    )

    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("biped_description"), "urdf", "biped_robot_12dof.urdf"]
        ),
        description="Absolute path to the robot URDF file",
    )

    reference_file_arg = DeclareLaunchArgument(
        "reference_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("biped_centroidal_mpc"), "config", "command", "reference.info"]
        ),
        description="Absolute path to the OCS2 reference.info file",
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu_sensor_broadcaster/imu",
        description="IMU topic to subscribe to",
    )

    joint_states_topic_arg = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/joint_states",
        description="Joint states topic to subscribe to",
    )

    update_rate_arg = DeclareLaunchArgument(
        "update_rate",
        default_value="500.0",
        description="State estimator update rate in Hz (should match controller_manager update_rate)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    # ── State estimator node ────────────────────────────────────────────────
    state_estimator_node = Node(
        package="legged_estimation",
        executable="state_estimator_node",
        name="state_estimator_node",
        output="both",
        parameters=[
            {
                "task_file":           LaunchConfiguration("task_file"),
                "urdf_file":           LaunchConfiguration("urdf_file"),
                "reference_file":      LaunchConfiguration("reference_file"),
                "imu_topic":           LaunchConfiguration("imu_topic"),
                "joint_states_topic":  LaunchConfiguration("joint_states_topic"),
                "update_rate":         LaunchConfiguration("update_rate"),
                "use_sim_time":        LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            task_file_arg,
            urdf_file_arg,
            reference_file_arg,
            imu_topic_arg,
            joint_states_topic_arg,
            update_rate_arg,
            use_sim_time_arg,
            state_estimator_node,
        ]
    )



