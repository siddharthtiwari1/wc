#!/usr/bin/env python3
"""
Launch file for CrowdSurfer-based crowd navigation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('wheelchair_crowd_navigation')

    # Declare arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to trained model checkpoint'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device for inference (cuda or cpu)'
    )

    planning_freq_arg = DeclareLaunchArgument(
        'planning_frequency',
        default_value='10.0',
        description='Planning frequency in Hz'
    )

    num_samples_arg = DeclareLaunchArgument(
        'num_samples',
        default_value='100',
        description='Number of trajectory samples for optimization'
    )

    # Crowd planner node
    crowd_planner_node = Node(
        package='wheelchair_crowd_navigation',
        executable='crowd_planner_node',
        name='crowd_planner',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'planning_frequency': LaunchConfiguration('planning_frequency'),
            'num_samples': LaunchConfiguration('num_samples'),
        }],
    )

    # Visualization node
    visualization_node = Node(
        package='wheelchair_crowd_navigation',
        executable='visualization_node',
        name='crowd_visualization',
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        planning_freq_arg,
        num_samples_arg,
        crowd_planner_node,
        visualization_node,
    ])
