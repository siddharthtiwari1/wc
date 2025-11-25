#!/usr/bin/env python3

"""
Dijkstra Path Planning WITH Localization
=========================================

Use this when localization is ALREADY running.
Only starts the Dijkstra planner - no duplicate map_server or RViz.

Usage:
------
Terminal 1: ros2 launch wheelchair_localization wheelchair_global_localization.launch.py
Terminal 2: ros2 launch wheelchair_planning dijkstra_with_localization.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheelchair_planning_dir = get_package_share_directory('wheelchair_planning')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    # Dijkstra planner node - subscribes to /map from localization's map_server
    dijkstra_planner_node = Node(
        package='wheelchair_planning',
        executable='dijkstra_planner.py',
        name='dijkstra_planner',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        dijkstra_planner_node,
    ])
