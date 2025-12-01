#!/usr/bin/env python3

"""
WHEELCHAIR FUSION NAVIGATION - COMPLETE AUTONOMOUS SYSTEM
==========================================================
Publication-quality autonomous navigation with visual-LiDAR fusion.

Architecture:
    wheelchair_fusion_localization.launch.py
        ├── Hardware Interface (Arduino + ros2_control)
        ├── Sensors (RPLidar S3, RealSense D455, IMU)
        ├── Robust Laser Filter (6-stage chain)
        ├── EKF Localization (wheel odom + IMU fusion)
        ├── Map Server (pre-built map)
        ├── AMCL (12k particles, 300 beams)
        └── Twist Converter (/cmd_vel → /wc_control/cmd_vel)

    + Nav2 Navigation Stack with STVL (THIS FILE)
        ├── Controller Server (Regulated Pure Pursuit)
        ├── Planner Server (SMAC Planner 2D)
        ├── Behavior Server (wheelchair-safe recovery)
        ├── BT Navigator (custom behavior tree)
        ├── Smoother Server (path smoothing)
        ├── Velocity Smoother (passenger comfort)
        ├── Collision Monitor (emergency safety)
        └── Waypoint Follower (multi-goal navigation)

Sensor Fusion:
    LOCAL COSTMAP:
        1. STVL Layer - D455 depth camera (3D obstacles, 4m range)
        2. Obstacle Layer - RPLidar S3 (360° coverage, 12m range)
        3. Inflation Layer - Safety buffer (55cm)

    GLOBAL COSTMAP:
        1. Static Layer - Pre-built map
        2. Obstacle Layer - RPLidar S3 dynamic obstacles
        3. Inflation Layer - Safety buffer

Usage:
    ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py

    With custom map:
    ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py \\
        map_name:=/path/to/map.yaml

Send navigation goals:
    1. RViz: Use "2D Goal Pose" button
    2. CLI:  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
             "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
    3. Python: Use nav2_simple_commander

Author: Siddharth Tiwari (s24035@students.iitmandi.ac.in)
Target: ICRA/IROS Publication
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
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
    default_nav2_params = os.path.join(
        wheelchair_navigation_dir, 'config', 'nav2_params_fusion.yaml'
    )
    default_bt_xml = os.path.join(
        wheelchair_navigation_dir, 'behavior_tree',
        'wheelchair_navigation_w_replanning_and_recovery.xml'
    )
    default_rviz_config = os.path.join(
        wheelchair_description_dir, 'rviz', 'fusion_navigation.rviz'
    )

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value=default_map_file,
        description='Full path to map YAML file'
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params', default_value=default_nav2_params,
        description='Full path to Nav2 parameters file'
    )
    declare_bt_xml = DeclareLaunchArgument(
        'bt_xml', default_value=default_bt_xml,
        description='Full path to behavior tree XML file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz visualization'
    )
    declare_use_collision_monitor = DeclareLaunchArgument(
        'use_collision_monitor', default_value='true',
        description='Enable collision monitor safety layer'
    )

    # Launch configurations
    map_name = LaunchConfiguration('map_name')
    nav2_params = LaunchConfiguration('nav2_params')
    bt_xml = LaunchConfiguration('bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')

    # ========================================================================
    # LOCALIZATION SYSTEM
    # ========================================================================
    # Includes: hardware, sensors, laser filter, EKF, AMCL, twist bridge

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheelchair_bringup_dir, 'launch',
                        'wheelchair_fusion_localization.launch.py')
        ),
        launch_arguments={
            'map_name': map_name,
            'use_sim_time': use_sim_time,
            'rviz': 'false',  # We use our own nav-enabled RViz config
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

    # Controller Server - Regulated Pure Pursuit
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav')  # Output to velocity smoother
        ]
    )

    # Planner Server - SMAC Planner 2D
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # Behavior Server - Wheelchair-safe recovery
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
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

    # Velocity Smoother - Passenger comfort
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),      # Input from controller
            ('cmd_vel_smoothed', '/cmd_vel'), # Output to collision monitor or bridge
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

    # Collision Monitor - Safety layer (optional)
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_collision_monitor),
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

    rviz_node = TimerAction(
        period=14.0,  # After localization and nav2 startup
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', default_rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
        condition=IfCondition(use_rviz),
    )

    # ========================================================================
    # STAGED NAVIGATION STARTUP
    # ========================================================================

    # Wait for localization to be fully ready before starting navigation
    nav2_startup = TimerAction(
        period=16.0,  # Wait for localization stack
        actions=[
            LogInfo(msg='[NAV2] Starting navigation stack with STVL fusion...'),
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            smoother_server,
            velocity_smoother,
            waypoint_follower,
        ]
    )

    collision_monitor_startup = TimerAction(
        period=18.0,
        actions=[collision_monitor],
    )

    nav2_lifecycle_startup = TimerAction(
        period=22.0,  # Wait for nav2 nodes to start
        actions=[
            LogInfo(msg='[NAV2] Starting lifecycle manager...'),
            nav2_lifecycle_manager,
        ]
    )

    # ========================================================================
    # READY MESSAGE
    # ========================================================================

    ready_message = TimerAction(
        period=28.0,
        actions=[
            LogInfo(msg='=' * 70),
            LogInfo(msg='  WHEELCHAIR FUSION NAVIGATION - SYSTEM READY'),
            LogInfo(msg='=' * 70),
            LogInfo(msg=''),
            LogInfo(msg='  SENSOR FUSION ACTIVE:'),
            LogInfo(msg='    - RPLidar S3: 360° LiDAR, 12m range'),
            LogInfo(msg='    - RealSense D455: Depth camera, 4m range'),
            LogInfo(msg='    - STVL: 3D voxel obstacle detection'),
            LogInfo(msg=''),
            LogInfo(msg='  NAVIGATION FEATURES:'),
            LogInfo(msg='    - SMAC Planner 2D (global path)'),
            LogInfo(msg='    - Regulated Pure Pursuit (local control)'),
            LogInfo(msg='    - Wheelchair-safe recovery behaviors'),
            LogInfo(msg='    - Passenger comfort velocity smoothing'),
            LogInfo(msg=''),
            LogInfo(msg='  SEND GOALS VIA:'),
            LogInfo(msg='    - RViz: "2D Goal Pose" button'),
            LogInfo(msg='    - CLI: ros2 action send_goal /navigate_to_pose ...'),
            LogInfo(msg=''),
            LogInfo(msg='=' * 70),
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
        declare_use_collision_monitor,

        # Localization system (hardware + sensors + EKF + AMCL)
        localization_launch,

        # RViz with navigation config
        rviz_node,

        # Nav2 navigation stack (staged startup)
        nav2_startup,
        collision_monitor_startup,
        nav2_lifecycle_startup,

        # Ready message
        ready_message,
    ])
