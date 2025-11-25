#!/usr/bin/env python3

"""
WHEELCHAIR AUTONOMOUS NAVIGATION - COMPLETE SYSTEM
===================================================
Full autonomous navigation stack for wheelchair.

Architecture:
    wheelchair_global_localization.launch.py
        ├── Hardware Interface (Arduino + ros2_control)
        ├── Sensors (RPLidar S3, RealSense D455, IMU)
        ├── EKF Localization (wheel odom + IMU fusion)
        ├── Map Server (pre-built map)
        ├── AMCL (global particle filter localization)
        ├── Depth to PointCloud2 (for 3D obstacle detection)
        └── Twist Converter (/cmd_vel → /wc_control/cmd_vel)

    + Nav2 Navigation Stack (THIS FILE)
        ├── Controller Server (DWB / Pure Pursuit / PD Motion)
        ├── Planner Server (NavFn / Dijkstra / A* custom)
        ├── Behavior Server (recovery: spin, backup, wait)
        ├── BT Navigator (full recovery behavior tree)
        ├── Smoother Server (path smoothing)
        ├── Velocity Smoother (jerk limiting for passenger comfort)
        ├── Collision Monitor (safety layer)
        └── Waypoint Follower (multi-goal navigation)

Obstacle Detection:
    - RPLidar S3: 360° 2D laser scan → /scan
    - RealSense D455: Front depth camera → /camera/depth/points (PointCloud2)
    - Both sources feed into costmaps for comprehensive obstacle detection

Custom Plugins Available:
    Planners (set in nav2_params_wheelchair.yaml):
        - nav2_navfn_planner/NavfnPlanner (default)
        - wheelchair_planning::DijkstraPlanner
        - wheelchair_planning::AStarPlanner

    Controllers (set in nav2_params_wheelchair.yaml):
        - dwb_core::DWBLocalPlanner (default)
        - wheelchair_motion::PurePursuit
        - wheelchair_motion::PDMotionPlanner

Usage:
    ros2 launch wheelchair_bringup wheelchair_autonomous_nav.launch.py

    With custom map:
    ros2 launch wheelchair_bringup wheelchair_autonomous_nav.launch.py map_name:=/path/to/map.yaml

Send navigation goals:
    1. RViz: Use "2D Goal Pose" button
    2. CLI:  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
             "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
    3. Python: Use nav2_simple_commander

Author: Wheelchair Autonomy Project
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    wheelchair_bringup_dir = get_package_share_directory('wheelchair_bringup')
    wheelchair_navigation_dir = get_package_share_directory('wheelchair_navigation')
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')

    # Default configurations
    default_map_file = '/home/sidd/wc/maps/my_map.yaml'
    default_nav2_params = os.path.join(wheelchair_navigation_dir, 'config', 'nav2_params_wheelchair.yaml')
    default_bt_xml = os.path.join(
        wheelchair_navigation_dir,
        'behavior_tree',
        'wheelchair_navigation_w_replanning_and_recovery.xml'
    )

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value=default_map_file,
        description='Full path to map YAML file'
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=default_nav2_params,
        description='Full path to Nav2 parameters file'
    )

    declare_bt_xml = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml,
        description='Full path to behavior tree XML file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    # Launch configurations
    map_name = LaunchConfiguration('map_name')
    nav2_params = LaunchConfiguration('nav2_params')
    bt_xml = LaunchConfiguration('bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    # ========================================================================
    # LOCALIZATION SYSTEM (includes hardware, sensors, EKF, AMCL, twist bridge)
    # ========================================================================

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheelchair_bringup_dir, 'launch', 'wheelchair_global_localization.launch.py')
        ),
        launch_arguments={
            'map_name': map_name,
            'use_sim_time': use_sim_time,
            'rviz': 'false',  # We'll use our own nav-enabled RViz config
        }.items()
    )

    # ========================================================================
    # NAV2 NAVIGATION STACK
    # ========================================================================

    # Nav2 lifecycle nodes
    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'smoother_server',
        'velocity_smoother',
        'waypoint_follower',
    ]

    # Controller Server - Local trajectory planning (DWB)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav')  # Output to velocity smoother input
        ]
    )

    # Planner Server - Global path planning (NavFn)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # Behavior Server - Recovery behaviors
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel')  # Recovery actions publish to /cmd_vel
        ]
    )

    # BT Navigator - Behavior tree execution
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time},
            {'default_nav_to_pose_bt_xml': bt_xml},
            {'default_nav_through_poses_bt_xml': bt_xml},
        ]
    )

    # Smoother Server - Path smoothing
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # Velocity Smoother - Smooth velocity commands for passenger comfort
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),         # Input from controller
            ('cmd_vel_smoothed', '/cmd_vel'),    # Output to twist converter → Arduino
        ]
    )

    # Waypoint Follower - Multi-goal navigation
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # Nav2 Lifecycle Manager
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': nav2_lifecycle_nodes},
            {'bond_timeout': 0.0},  # Disable bond for easier debugging
        ]
    )

    # ========================================================================
    # RVIZ WITH NAVIGATION CONFIG
    # ========================================================================

    # RViz config for navigation (includes costmaps, paths, Nav2 displays)
    nav_rviz_config = os.path.join(
        wheelchair_description_dir,
        'rviz',
        'navigation.rviz'
    )

    rviz_node = TimerAction(
        period=12.0,  # After localization RViz would have started, but we disabled it
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', nav_rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
        condition=IfCondition(use_rviz),
    )

    # ========================================================================
    # STAGED NAVIGATION STARTUP (after localization is ready)
    # ========================================================================

    # Wait for localization to be fully ready before starting navigation
    nav2_startup = TimerAction(
        period=15.0,  # Wait for localization stack to initialize
        actions=[
            LogInfo(msg='[NAV2] Starting navigation stack...'),
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            smoother_server,
            velocity_smoother,
            waypoint_follower,
        ]
    )

    nav2_lifecycle_startup = TimerAction(
        period=20.0,  # Wait for nav2 nodes to start
        actions=[
            LogInfo(msg='[NAV2] Starting lifecycle manager...'),
            nav2_lifecycle_manager,
        ]
    )

    ready_message = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='========================================'),
            LogInfo(msg='  WHEELCHAIR AUTONOMOUS NAVIGATION'),
            LogInfo(msg='  SYSTEM READY'),
            LogInfo(msg='========================================'),
            LogInfo(msg='Send goals via:'),
            LogInfo(msg='  - RViz "2D Goal Pose" button'),
            LogInfo(msg='  - ros2 action send_goal /navigate_to_pose ...'),
            LogInfo(msg='========================================'),
        ]
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        declare_map_name,
        declare_nav2_params,
        declare_bt_xml,
        declare_use_sim_time,
        declare_autostart,
        declare_use_rviz,

        # Localization system (hardware + sensors + EKF + AMCL + twist bridge)
        localization_launch,

        # RViz with navigation config
        rviz_node,

        # Nav2 navigation stack (staged startup)
        nav2_startup,
        nav2_lifecycle_startup,
        ready_message,
    ])
