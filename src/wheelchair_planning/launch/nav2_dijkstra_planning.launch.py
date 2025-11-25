#!/usr/bin/env python3

"""
Nav2 Dijkstra Path Planning Launch File (C++ Plugin)
=====================================================

Uses your custom C++ Dijkstra planner plugin with Nav2.

Prerequisites:
1. Localization must be running first:
   ros2 launch wheelchair_localization wheelchair_global_localization.launch.py

2. Then run this:
   ros2 launch wheelchair_planning nav2_dijkstra_planning.launch.py

To test:
- In RViz, use "2D Goal Pose" button to set a goal
- Path appears on /plan topic
- Or call action: ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "..."
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
        'nav2_planning.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Planner Server - runs the C++ Dijkstra plugin
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Simple goal relay - publishes path when goal_pose received
    # This bridges /goal_pose to the planner action
    goal_to_plan_node = Node(
        package='wheelchair_planning',
        executable='goal_to_plan.py',
        name='goal_to_plan',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Lifecycle manager
    lifecycle_manager_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_planning',
                output='screen',
                parameters=[
                    {
                        'node_names': [
                            'planner_server',
                            'smoother_server',
                        ],
                        'autostart': True,
                        'use_sim_time': use_sim_time,
                        'bond_timeout': 10.0,
                    }
                ],
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        planner_server_node,
        smoother_server_node,
        goal_to_plan_node,
        lifecycle_manager_node,
    ])
