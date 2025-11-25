#!/usr/bin/env python3

"""
Path Planning Test Launch File
==============================

Tests Dijkstra path planner with your localization system.

Prerequisites:
1. Have a map created (using mapping mode)
2. Localization should be running

Usage:
------
ros2 launch wheelchair_planning test_dijkstra_planning.launch.py \
    map_file:=/home/sidd/wc/maps/my_map.yaml

This will:
1. Start the map server with your map
2. Launch the Dijkstra planner node
3. Launch RViz with planning visualization

To test:
1. Set an initial pose in RViz (2D Pose Estimate) if needed
2. Set a goal pose using "2D Goal Pose" tool in RViz
3. Watch the Dijkstra algorithm explore the space (blue cells)
4. See the planned path (green line)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    wheelchair_planning_dir = get_package_share_directory('wheelchair_planning')

    # Default map file
    default_map_file = os.path.join(
        os.path.expanduser('~'),
        'wc',
        'maps',
        'my_map.yaml'
    )

    # Launch arguments
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Path to map yaml file for planning',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    # Map server (publishes map for costmap)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': LaunchConfiguration('map_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    # Lifecycle manager for map server
    map_server_lifecycle = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                output='screen',
                parameters=[
                    {
                        'node_names': ['map_server'],
                        'autostart': True,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ]
            )
        ]
    )

    # Costmap node (converts /map to /costmap for planner)
    costmap_node = TimerAction(
        period=2.0,  # Wait for map server to start
        actions=[
            Node(
                package='nav2_costmap_2d',
                executable='nav2_costmap_2d',
                name='costmap_node',
                output='screen',
                parameters=[
                    os.path.join(
                        wheelchair_planning_dir,
                        'config',
                        'costmap.yaml'
                    ),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ],
                remappings=[
                    ('/global_costmap/costmap', '/costmap'),
                ]
            )
        ]
    )

    # Dijkstra planner node
    dijkstra_planner_node = TimerAction(
        period=4.0,  # Wait for costmap to start
        actions=[
            Node(
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
        ]
    )

    # RViz for visualization
    rviz_config_file = os.path.join(
        wheelchair_planning_dir,
        'rviz',
        'planning_test.rviz'
    ) if os.path.exists(os.path.join(wheelchair_planning_dir, 'rviz', 'planning_test.rviz')) else ''

    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file] if rviz_config_file else [],
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ]
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        declare_map_file,
        declare_use_sim_time,

        # Nodes
        map_server_node,
        map_server_lifecycle,
        costmap_node,
        dijkstra_planner_node,
        rviz_node,
    ])
