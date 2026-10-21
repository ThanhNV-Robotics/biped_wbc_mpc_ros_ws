"""Visualize the biped URDF in RViz.

Data flow:

  joint_state_publisher(_gui) --/joint_states--> robot_state_publisher --/tf-----------------+
    (joint angles)                                 (URDF + angles -> link poses)             |
                                                   +--/robot_description (URDF text)--+      |
  static_transform_publisher --/tf_static (world -> link0_torso)----------------------+------+
                                                                                      v      v
                                                                                      rviz2

Usage:
  ros2 launch biped_description display.py
  ros2 launch biped_description display.py model:=biped_robot_12dof.urdf base_height:=0.80 gui:=false

Debug:
  ros2 topic echo /joint_states --once                    # angles from the GUI
  ros2 run tf2_ros tf2_echo world left_ankle_roll_link    # final foot pose
  ros2 run rqt_tf_tree rqt_tf_tree                        # whole frame tree
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('biped_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')

    # Run the URDF through xacro so <xacro:arg> can be added later without touching this file
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([pkg_share, 'urdf', model]),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'model',
            default_value='v2_biped_robot_12dof.urdf',
            description='URDF/xacro file name inside biped_description/urdf'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start joint_state_publisher_gui (sliders) instead of joint_state_publisher'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz']),
            description='Absolute path to the RViz config file'),
        DeclareLaunchArgument(
            'base_height',
            default_value='0.86',
            description='Height of link0_torso above the world frame [m]'),

        # robot_state_publisher: joint angles -> link poses (forward kinematics)
        #   in : robot_description parameter (URDF text after xacro), /joint_states
        #   out: /robot_description  URDF as a latched topic, RViz loads the meshes from it
        #        /tf                 one transform per moving joint
        #                            (link0_torso -> left_hip_yaw_link -> ... -> left_ankle_roll_link)
        #        /tf_static          transforms of fixed joints (none in this URDF)
        # On the real robot this node stays; only the /joint_states source changes
        # (joint_state_broadcaster fed by the STM32 instead of the sliders).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }]),

        # joint_state_publisher_gui (gui:=true): one slider per non-fixed joint, range taken
        # from <limit lower upper> in the URDF; publishes sensor_msgs/JointState on
        # /joint_states at ~10 Hz. Use it to check joint axes, signs and limits by hand.
        #
        # joint_state_publisher (gui:=false): same without the window; publishes zeros
        # (or initial values). Use it when you only need the robot to appear.
        #
        # IfCondition/UnlessCondition guarantee exactly one of them runs - two publishers
        # on /joint_states would fight each other.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui)),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(gui)),

        # rviz2: 3D viewer only, computes nothing.
        #   -d rviz_config  loads rviz/display.rviz (Fixed Frame = world, Grid, RobotModel, TF)
        #   RobotModel      meshes from /robot_description, each link placed using /tf
        #   use_sim_time    must match the simulator (MuJoCo later), otherwise TF is rejected
        #                   as too old / from the future
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]),
            
        # static_transform_publisher: world -> link0_torso at z = base_height, published once
        # on the latched /tf_static, so nodes started later still receive it.
        # Without it link0_torso is the TF root and the torso sits at the grid origin.
        # On the real robot REMOVE this node - the state estimator publishes a moving
        # world -> link0_torso on /tf, and two publishers of the same transform flicker.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_torso',
            output='screen',
            arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('base_height'),
                    '--frame-id', 'world', '--child-frame-id', 'link0_torso']),

    ])
