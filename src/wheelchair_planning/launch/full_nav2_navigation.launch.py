#!/usr/bin/env python3

"""
Full Nav2 Navigation Stack with Custom Dijkstra Planner
========================================================

This launches the COMPLETE Nav2 stack including:
- bt_navigator (handles /goal_pose from RViz)
- planner_server (path planning with your Dijkstra plugin)
- controller_server (path following with DWB)
- behavior_server (recovery behaviors)
- smoother_server (path smoothing)

Prerequisites:
1. Localization must be running:
   ros2 launch wheelchair_localization wheelchair_global_localization.launch.py

2. Then run this:
   ros2 launch wheelchair_planning full_nav2_navigation.launch.py

To test:
- In RViz, click "2D Goal Pose" and set a goal on the map
- The wheelchair will plan and navigate to the goal
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheelchair_planning_dir = get_package_share_directory('wheelchair_planning')

    nav2_params_file = os.path.join(
        wheelchair_planning_dir,
        'config',
        'nav2_full_params.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Controller Server - follows the planned path
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # Planner Server - plans global paths (uses your Dijkstra plugin)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # Smoother Server - smooths planned paths
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # Behavior Server - recovery behaviors (spin, backup, wait)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # BT Navigator - the brain that coordinates everything
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # Lifecycle manager - activates all nodes
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
    ]

    lifecycle_manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'node_names': lifecycle_nodes},
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'bond_timeout': 10.0},
                ],
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
