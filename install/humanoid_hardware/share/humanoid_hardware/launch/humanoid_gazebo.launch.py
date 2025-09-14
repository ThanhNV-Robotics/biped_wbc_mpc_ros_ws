import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('humanoid_hardware')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    set_gz_paths = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            '/usr/share/gz-garden/models',
            '/usr/share/gz-garden/worlds',
            os.path.expanduser('~/.gz/models'),
            os.path.expanduser('~/.gazebo/models'),
            os.path.join(pkg_path, 'worlds'),
            os.path.join(pkg_path, 'models'),
        ])
    )

    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    xacro_file = os.path.join(pkg_path, 'urdf', 'humanoid.gazebo.xacro')
    controllers_yaml = os.path.join(pkg_path, 'config', 'humanoid_controllers_gz.yaml')

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'controllers_yaml': controllers_yaml}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'humanoid',
                   '-x', '0.0', '-y', '0.0', '-z', '0.81',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_cmd', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_then_spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                effort_controller_spawner,
                imu_sensor_broadcaster_spawner,
            ],
        )
    )

    return LaunchDescription([
        set_gz_paths,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_then_spawn_controllers,
    ])
