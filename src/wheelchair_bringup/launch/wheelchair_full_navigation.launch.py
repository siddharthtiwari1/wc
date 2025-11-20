#!/usr/bin/env python3

"""
Complete Wheelchair System with Autonomous Navigation
======================================================

Launches the entire wheelchair system including hardware, sensors, localization,
SLAM mapping, AND Nav2 autonomous navigation stack.

This is the ULTIMATE launch file combining:
1. wheelchair_full_system.launch.py (hardware + sensors + EKF localization)
2. wheelchair_slam.launch.py (SLAM mapping)
3. wheelchair_navigation/navigation.launch.py (Nav2 autonomous navigation)

Usage:
    Real hardware with navigation:
        ros2 launch wheelchair_bringup wheelchair_full_navigation.launch.py \\
            is_sim:=false port:=/dev/ttyACM0 enable_navigation:=true

    Localization mode (use existing map):
        ros2 launch wheelchair_bringup wheelchair_full_navigation.launch.py \\
            is_sim:=false port:=/dev/ttyACM0 enable_navigation:=true \\
            use_slam:=false map:=/path/to/map.yaml

    SLAM + Navigation (simultaneous mapping and navigation):
        ros2 launch wheelchair_bringup wheelchair_full_navigation.launch.py \\
            is_sim:=false port:=/dev/ttyACM0 enable_navigation:=true \\
            use_slam:=true

Architecture:
    Hardware Layer:
        - Wheelchair motors + encoders (ros2_control)
        - RealSense D455 (RGB-D + IMU)
        - RPLidar S3 (2D laser)

    Localization Layer:
        - EKF Local: fuses wheel odometry + IMU → /odometry/filtered
        - EKF Global: fuses local odom + SLAM → /odometry/global
        - AMCL (optional): particle filter localization with map

    Mapping Layer (optional):
        - SLAM Toolbox: creates and updates map from laser scans

    Navigation Layer:
        - Planner Server: global path planning (A*)
        - Controller Server: local trajectory planning (DWB)
        - Behavior Server: recovery behaviors
        - Smoother Server: path smoothing
        - Velocity Smoother: smooth velocity commands
        - Collision Monitor: emergency stop safety
        - BT Navigator: behavior tree executive
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Get package directories
    wheelchair_bringup_dir = get_package_share_directory('wheelchair_bringup')
    wheelchair_navigation_dir = get_package_share_directory('wheelchair_navigation')
    wheelchair_mapping_dir = get_package_share_directory('wheelchair_mapping')

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    # From wheelchair_full_system.launch.py
    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='True for simulation, false for real hardware'
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for wheelchair hardware interface'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    declare_enable_plotting = DeclareLaunchArgument(
        'enable_plotting',
        default_value='false',  # Disable test plotting when navigating
        description='Enable square path EKF plotting (disable for navigation)'
    )

    # Navigation-specific arguments
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable Nav2 autonomous navigation stack'
    )

    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Use SLAM for mapping (true) or localization with existing map (false)'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file (required if use_slam:=false)'
    )

    declare_autostart_nav = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the navigation stack'
    )

    # ========================================================================
    # INCLUDE WHEELCHAIR FULL SYSTEM
    # ========================================================================
    # This includes:
    # - Hardware interface + ros2_control
    # - All sensors (RealSense, RPLidar, IMU)
    # - EKF localization (local + global)
    # - RViz visualization

    wheelchair_full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wheelchair_bringup_dir,
                'launch',
                'wheelchair_full_system.launch.py'
            )
        ]),
        launch_arguments={
            'is_sim': LaunchConfiguration('is_sim'),
            'port': LaunchConfiguration('port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
            'enable_plotting': LaunchConfiguration('enable_plotting'),
        }.items()
    )

    # ========================================================================
    # INCLUDE SLAM (if use_slam:=true)
    # ========================================================================
    # SLAM Toolbox for simultaneous mapping and localization

    wheelchair_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wheelchair_mapping_dir,
                'launch',
                'wheelchair_slam.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    # ========================================================================
    # MAP SERVER (if use_slam:=false)
    # ========================================================================
    # Load pre-existing map for localization mode

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': LaunchConfiguration('map')}
        ],
        condition=IfCondition(LaunchConfiguration('use_slam'))  # Only if NOT using SLAM
    )

    # Map server lifecycle manager
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server']}
        ],
        condition=IfCondition(LaunchConfiguration('use_slam'))  # Only if NOT using SLAM
    )

    # ========================================================================
    # INCLUDE NAV2 NAVIGATION STACK
    # ========================================================================
    # Full Nav2 autonomous navigation with all components

    wheelchair_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wheelchair_navigation_dir,
                'launch',
                'navigation.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_navigation'))
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        declare_is_sim,
        declare_port,
        declare_use_sim_time,
        declare_rviz,
        declare_enable_plotting,
        declare_enable_navigation,
        declare_use_slam,
        declare_map,
        declare_autostart_nav,

        # Core system (hardware + sensors + localization)
        wheelchair_full_system,

        # Mapping/Localization layer
        wheelchair_slam,
        map_server,
        map_lifecycle_manager,

        # Navigation layer
        wheelchair_navigation,
    ])
