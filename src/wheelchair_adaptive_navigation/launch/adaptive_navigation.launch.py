#!/usr/bin/env python3
"""Launch file for adaptive crowd navigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to trained diffusion model'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device (cuda or cpu)'
    )

    # Adaptive planner node
    planner_node = Node(
        package='wheelchair_adaptive_navigation',
        executable='adaptive_planner_node',
        name='adaptive_planner',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'ensemble_size': 10,
        }],
    )

    # Social visualization node
    viz_node = Node(
        package='wheelchair_adaptive_navigation',
        executable='social_viz_node',
        name='social_viz',
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        planner_node,
        viz_node,
    ])
