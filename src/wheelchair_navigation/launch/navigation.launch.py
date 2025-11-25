#!/usr/bin/env python3

"""
Wheelchair Navigation Stack Launch File
========================================

Launches Nav2 navigation stack for autonomous wheelchair navigation.
Based on robust bumperbot_ws navigation architecture.

Components:
- Controller Server (DWB/Pure Pursuit/PD Motion controller)
- Planner Server (NavFn/Dijkstra/A* global planner)
- Behavior Server (recovery behaviors)
- BT Navigator (behavior tree executive with full recovery)
- Smoother Server (path smoothing)
- Velocity Smoother (smooth velocity commands)
- Collision Monitor (safety layer)
- Waypoint Follower (multi-goal navigation)
- Lifecycle Manager (node management)

Custom Plugins Available:
- Planners: wheelchair_planning::DijkstraPlanner, wheelchair_planning::AStarPlanner
- Controllers: wheelchair_motion::PurePursuit, wheelchair_motion::PDMotionPlanner

Usage:
    With existing map (localization mode):
        ros2 launch wheelchair_navigation navigation.launch.py use_sim_time:=false

    With custom behavior tree:
        ros2 launch wheelchair_navigation navigation.launch.py bt_xml:=/path/to/bt.xml

    With simulation:
        ros2 launch wheelchair_navigation navigation.launch.py use_sim_time:=true

Prerequisites:
- Map must be published on /map topic (from SLAM or map_server)
- Localization must be running (/odometry/filtered or AMCL)
- Robot description must be available (URDF/transforms)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get package directories
    wheelchair_navigation_pkg = get_package_share_directory('wheelchair_navigation')

    # Nav2 parameters file
    nav2_params_file = os.path.join(
        wheelchair_navigation_pkg,
        'config',
        'nav2_params_wheelchair.yaml'
    )

    # Default behavior tree with full recovery
    default_bt_xml = os.path.join(
        wheelchair_navigation_pkg,
        'behavior_tree',
        'wheelchair_navigation_w_replanning_and_recovery.xml'
    )

    # Lifecycle nodes managed by lifecycle_manager
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'smoother_server',
        'velocity_smoother',
        'collision_monitor',
        'waypoint_follower'
    ]

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to Nav2 parameters file'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_bt_xml = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml,
        description='Full path to behavior tree XML file'
    )

    # ========================================================================
    # NAV2 NODES
    # ========================================================================

    # Controller Server - Local trajectory planning and control
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav')  # Remap to avoid conflict with teleop
        ]
    )

    # Planner Server - Global path planning
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Behavior Server - Recovery behaviors (spin, backup, wait)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # BT Navigator - Behavior tree based navigation executive with full recovery
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'default_nav_to_pose_bt_xml': LaunchConfiguration('bt_xml')}
        ]
    )

    # Smoother Server - Path smoothing for better trajectories
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Velocity Smoother - Smooth velocity commands for passenger comfort
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ]
    )

    # Collision Monitor - Emergency stop on imminent collision
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Waypoint Follower - Multi-goal navigation
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Lifecycle Manager - Manages lifecycle nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': lifecycle_nodes}
        ]
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_bt_xml,

        # Nav2 nodes
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        smoother_server,
        velocity_smoother,
        collision_monitor,
        waypoint_follower,
        lifecycle_manager,
    ])
