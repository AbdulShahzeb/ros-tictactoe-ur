#!/usr/bin/env python3
"""
Launch file for TicTacToe game node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
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

    enable_robot_arg = DeclareLaunchArgument(
        'enable_robot',
        default_value='false',
        description='Enable robot arm for physical gameplay'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera for board detection'
    )

    ui_enabled_arg = DeclareLaunchArgument(
        'ui_enabled',
        default_value='true',
        description='Enable pygame UI for visualization'
    )

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
            'enable_robot': LaunchConfiguration('enable_robot'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'ui_enabled': LaunchConfiguration('ui_enabled'),
        }]
    )

    return LaunchDescription([
        player_arg,
        agent_x_file_arg,
        agent_o_file_arg,
        enable_robot_arg,
        enable_camera_arg,
        ui_enabled_arg,
        tictactoe_node,
    ])