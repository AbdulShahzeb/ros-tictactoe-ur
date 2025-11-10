#!/usr/bin/env python3
"""
Launch file for TicTacToe game node.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


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

    # RealSense
    '''realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('realsense2_camera').find('realsense2_camera'), 'launch'),
            '/rs_launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )'''

    # TicTacToe node
    tictactoe_node = Node(
        package='brain',
        executable='tictactoe_node',
        name='tictactoe_node',
        output='screen',
        parameters=[{
            'player': LaunchConfiguration('player'),
            'agent_x_file': LaunchConfiguration('agent_x_file'),
            'agent_o_file': LaunchConfiguration('agent_o_file'),
            'enable_serial': False
        }]
    )

    # Grid Vision Node
    grid_vision_node = Node(
        package='perception',
        executable='grid_vision_node',
        name='grid_vision_node',
        output='screen',
        parameters=[{
            'exposure': 90,
            'corner0_x': 0.8315,
            'corner0_y': -0.0909,
            'corner1_x': 0.8315,
            'corner1_y': 0.5000,
            'corner2_x': 0.2625,
            'corner2_y': -0.0912,
            'corner3_x': 0.2625,
            'corner3_y': 0.5000
        }]
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

    return LaunchDescription([
        player_arg,
        agent_x_file_arg,
        agent_o_file_arg,
        tictactoe_node,
        keyboard_node,
        grid_vision_node,
        #realsense_launch,
        static_transform_node,
    ])