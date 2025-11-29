#!/usr/bin/env python3
"""
Launch file for Perception nodes.
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ArUco Vision Node
    aruco_vision_node = Node(
        package='perception',
        executable='aruco_vision_node',
        name='aruco_vision_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Cell Vision Node
    cell_vision_node = Node(
        package='perception',
        executable='cell_vision_node',
        name='cell_vision_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        aruco_vision_node,
        cell_vision_node,
    ])