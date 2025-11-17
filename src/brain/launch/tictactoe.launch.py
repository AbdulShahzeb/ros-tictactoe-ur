#!/usr/bin/env python3
"""
Launch file for TicTacToe game node.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    # Delay for Brain
    delay_brain = TimerAction(
        period=3.0,
        actions=[brain_node]
    )

    return LaunchDescription([
        player_arg,
        agent_x_file_arg,
        agent_o_file_arg,
        delay_brain,
        keyboard_node,
        grid_vision_node,
        static_transform_node,
    ])