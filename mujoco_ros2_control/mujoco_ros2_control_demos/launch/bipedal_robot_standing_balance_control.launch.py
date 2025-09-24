import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    print("hello")

    ###############################################
    # Neccessary paths
    ###############################################
    mujoco_ros2_control_demos_path = os.path.join(
        get_package_share_directory('mujoco_ros2_control_demos'))
    
    # define the controller yaml config file
    # controller_config_file = os.path.join(mujoco_ros2_control_demos_path, 'config', 'bipedal_robot_position_trajectory_cotroller.yaml')
    
    # Get the YAML config file path
    standing_balance_controller_cfg_file = PathJoinSubstitution(
        [
            FindPackageShare("control_actions"),
            "config",
            "StandingBalanceController.yaml",
        ]
    )
    ###############################################
    # Process urdf/xacro file
    ###############################################    
    xacro_file = os.path.join(mujoco_ros2_control_demos_path,
                              'urdf',
                              'bipedal_robot_main_simu.urdf') # read urdf file

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}# convert to xml

    ###############################################
    # Node management
    ############################################### 
    # mujoco simulator node 
    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            standing_balance_controller_cfg_file,
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Bipedal_Robot.xml')}
        ]
    ) 
    # standing balance controller node
    # remark: use the Bipedal_Robot_Mimic.xml, with a fixed torso, to compute some kinematic values only
    node_standing_balance_controller = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[standing_balance_controller_cfg_file, 
                        {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Bipedal_Robot_Mimic.xml')}
                        ],
            output="screen",
        )
    
    # control actions server node
    node_control_action = Node(
            package='control_actions',
            executable='control_actions_server_node',
            output='screen',
            parameters=[
                {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Bipedal_Robot.xml')}
            ]
        ) 
    
    # mujoco mimic node
    node_mujoco_ros2_mimic = Node(
        package='robot_state_visualization',
        executable='robot_state_visualization_node',
        output='screen',
        parameters=[
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Bipedal_Robot_Mimic.xml')}
        ]
    ) 
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    ) # robot_state_publisher node

    ###############################################
    # Load controllers
    ###############################################

    # First we use joint_trajectory_controller to perform some basic movement functions
    load_joint_position_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_trajectory_controller'],
            output='screen'
        ) # at the beginning, load the joint_trajectory_controller and set to active to claim the state interface  
    
    # the standing balance controller will be loaded but not activated at the beginning
    standing_balance_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'standing_balance_controller'],
        output='screen'
    )

    # publish joint states (position and velocity), load this controller and set to active
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    ) 
    
    load_imu_sensor_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'imu_sensor_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_broadcaster],
            )
        ),
        # Add a 2 second delay after loading joint state broadcaster before loading the next controller
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=load_joint_state_broadcaster,
                on_start=[
                    TimerAction(
                        period=2.0,  # 2 second delay
                        actions=[load_imu_sensor_broadcaster]
                    )
                ],
            )
        ),
        # Register joint_trajectory_controller after imu_sensor_broadcaster is loaded
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=load_imu_sensor_broadcaster,
                on_start=[load_joint_position_controller]
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_imu_sensor_broadcaster,
                on_exit=[
                    TimerAction(
                        period=1.0, # 1 second delay
                        actions=[standing_balance_controller_spawner]
                    )
                ],
            )
        ),

        node_mujoco_ros2_control,
        node_robot_state_publisher,
        node_standing_balance_controller,
        # standing_balance_controller_spawner,
        node_mujoco_ros2_mimic,
        node_control_action
    ])