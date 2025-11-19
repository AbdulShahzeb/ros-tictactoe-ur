#!/usr/bin/env python3
"""
Launch file for TicTacToe game node.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('brain')
    
    # Default paths for agent files
    default_agent_x = os.path.join(pkg_dir, 'models', 'menace_agent_x.npy')
    default_agent_o = os.path.join(pkg_dir, 'models', 'menace_agent_o.npy')

    # Declare launch arguments
    player_arg = DeclareLaunchArgument(
        'player',
        default_value='x',
        description='Which symbol to play as: x or o'
    )

    agent_x_file_arg = DeclareLaunchArgument(
        'agent_x_file',
        default_value=default_agent_x,
        description='Path to X agent model file'
    )

    agent_o_file_arg = DeclareLaunchArgument(
        'agent_o_file',
        default_value=default_agent_o,
        description='Path to O agent model file'
    )

    use_fake = False
    ip_address = "192.168.56.101"
    if not use_fake:
        ip_address = "192.168.0.100"
    description_file = os.path.join(
        get_package_share_directory('end_effector'),
        'urdf',
        'ur_with_pen_holder.xacro'
    )
    kinematics_params_file = os.path.join(
        get_package_share_directory('brain'),
        'config',
        'robot1_calib.yaml'
    )


    # Launch UR driver
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': ip_address,
            'launch_rviz': 'false',
            'description_file': description_file,
            'kinematics_params': kinematics_params_file,
        }.items()
    )

    # Launch MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': ip_address,
            'launch_rviz': 'true',
        }.items()
    )

    # Delay MoveIt
    delay_moveit = TimerAction(
        period=3.0,
        actions=[moveit_launch]
    )


    # Brain node
    brain_node = Node(
        package='brain',
        executable='brain_node',
        name='brain_node',
        output='screen',
        parameters=[{
            'player': LaunchConfiguration('player'),
            'agent_x_file': LaunchConfiguration('agent_x_file'),
            'agent_o_file': LaunchConfiguration('agent_o_file'),
        }]
    )

    # Aruco Vision Node
    aruco_vision_node = Node(
        package='perception',
        executable='aruco_vision_node',
        name='aruco_vision_node',
        output='screen',
        parameters=[{
            'exposure': 120
        }]
    )

    # Cell Vision Node
    cell_vision_node = Node(
        package='perception',
        executable='cell_vision_node',
        name='cell_vision_node',
        output='screen',
    )

    # Static Transform
    static_transform_node = Node(
        package='manipulation',
        executable='static_transform',
        name='static_transform_node',
        output='screen',
    )

    # Keyboard Node
    keyboard_node = Node(
        package='brain',
        executable='keyboard_node',
        name='keyboard_node',
        output='screen',
        emulate_tty=True,
        prefix='xterm -e'
    )

    # Delay for Brain
    delay_brain = TimerAction(
        period=3.0,
        actions=[brain_node]
    )

    return LaunchDescription([
        player_arg,
        agent_x_file_arg,
        agent_o_file_arg,
        ur_control_launch,
        delay_moveit,
        delay_brain,
        keyboard_node,
        aruco_vision_node,
        cell_vision_node,
        static_transform_node,
    ])