#!/usr/bin/env python3

"""
═══════════════════════════════════════════════════════════════════════════════
WHEELCHAIR MAPPING SYSTEM - TOP CLASS
═══════════════════════════════════════════════════════════════════════════════

Complete mapping system for autonomous wheelchair navigation.
Optimized for RPLidar 40m + RealSense D455 IMU + Superior EKF Odometry.

SINGLE COMMAND TO CREATE HIGH-QUALITY MAPS!

Usage:
    ros2 launch wheelchair_bringup wheelchair_mapping_top_class.launch.py

What This Launches:
    ✓ Hardware interface (motors, controllers)
    ✓ RPLidar 40m driver (/scan topic)
    ✓ RealSense D455 camera + IMU
    ✓ EKF sensor fusion → /odometry/filtered
    ✓ SLAM Toolbox (wheelchair-optimized config, async mode)
    ✓ Map saver server
    ✓ Lifecycle manager (slam_toolbox + map_saver_server with autostart)
    ✓ Teleop (joystick control)
    ✓ RViz (visualization)

Lifecycle Management:
    • lifecycle_manager_slam: Manages slam_toolbox + map_saver_server
    • Set to autostart=True for automatic activation
    • ROS distro aware (slam_toolbox lifecycle differs in Humble vs Jazzy)

After Mapping:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_wheelchair_map

Author: Advanced Wheelchair Navigation System
Status: PRODUCTION READY
Architecture: Based on bumperbot reference with proper lifecycle management
═══════════════════════════════════════════════════════════════════════════════
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')
    wheelchair_localization_dir = get_package_share_directory('wheelchair_localization')
    wc_control_dir = get_package_share_directory('wc_control')

    # Default paths
    default_model_path = os.path.join(
        wheelchair_description_dir,
        'urdf',
        'wheelchair_description.urdf.xacro',
    )
    default_rviz_config = os.path.join(
        wheelchair_description_dir,
        'rviz',
        'urdf_config.rviz',
    )

    # Detect ROS distro
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='True for simulation, false for real hardware.',
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for wheelchair hardware interface.',
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to wheelchair URDF/xacro file.',
    )

    declare_is_ignition = DeclareLaunchArgument(
        'is_ignition',
        default_value=default_is_ignition,
        description='Set to true for Humble/ros_ign.',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.',
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file.',
    )

    declare_unite_imu = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='RealSense IMU synchronization.',
    )

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar S3.',
    )

    declare_sudo_password = DeclareLaunchArgument(
        'sudo_password',
        default_value='12345',
        description='Sudo password for USB permissions.',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz.',
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz config file.',
    )

    is_sim = LaunchConfiguration('is_sim')

    # ========================================================================
    # USB PERMISSIONS
    # ========================================================================

    def _grant_usb_permissions(context, *_args, **_kwargs):
        """Grant USB permissions for hardware."""
        lidar_port = LaunchConfiguration('lidar_port').perform(context)
        wheelchair_port = LaunchConfiguration('port').perform(context)
        password = LaunchConfiguration('sudo_password').perform(context)

        ports = [wheelchair_port]
        if os.path.exists(lidar_port):
            ports.append(lidar_port)

        for port in ports:
            try:
                subprocess.run(
                    ['sudo', '-S', 'chmod', '666', port],
                    input=f'{password}\n',
                    text=True,
                    check=True,
                )
                print(f'✓ Updated permissions for {port}')
            except subprocess.CalledProcessError as exc:
                print(f'⚠ Failed to set permissions for {port}: {exc}')
        return []

    permission_setup = OpaqueFunction(
        function=_grant_usb_permissions,
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # HARDWARE & SENSORS
    # ========================================================================

    # Unified wheelchair hardware + control
    unified_wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(wc_control_dir, 'launch', 'unified_wheelchair.launch.py')
        ]),
        launch_arguments={
            'is_sim': LaunchConfiguration('is_sim'),
            'port': LaunchConfiguration('port'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Sensors (RealSense + IMU)
    wheelchair_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(wc_control_dir, 'launch', 'wheelchair_sensors.launch.py')
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'is_ignition': LaunchConfiguration('is_ignition'),
            'is_sim': LaunchConfiguration('is_sim'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
        condition=UnlessCondition(is_sim)
    )

    # RPLidar S3
    rplidar_s3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s3_launch.py',
            )
        ]),
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # SENSOR FUSION (EKF)
    # ========================================================================

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu"]
    )

    # Local EKF (your perfectly working odometry!)
    ekf_local_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    os.path.join(wheelchair_localization_dir, 'config', 'ekf.yaml'),
                    {'use_sim_time': is_sim}
                ]
            )
        ]
    )

    # ========================================================================
    # SLAM TOOLBOX - WHEELCHAIR OPTIMIZED
    # ========================================================================

    slam_toolbox_mapping = TimerAction(
        period=8.0,  # Wait for sensors to initialize
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(
                        wheelchair_localization_dir,
                        'config',
                        'slam_toolbox_wheelchair_optimized.yaml',
                    ),
                    {'use_sim_time': is_sim}
                ]
            )
        ]
    )

    # Map saver server
    map_saver_server = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver_server',
                output='screen',
                parameters=[
                    {'save_map_timeout': 5.0},
                    {'use_sim_time': is_sim},
                    {'free_thresh_default': 0.196},
                    {'occupied_thresh_default': 0.65},
                ]
            )
        ]
    )

    # Lifecycle manager
    # Note: In ROS Jazzy, slam_toolbox is a lifecycle node
    # In ROS Humble, it's not (managed internally)
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    lifecycle_nodes = ['map_saver_server']
    if ros_distro != 'humble':
        lifecycle_nodes.append('slam_toolbox')

    lifecycle_manager = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[
                    {'node_names': lifecycle_nodes},
                    {'use_sim_time': is_sim},
                    {'autostart': True}
                ]
            )
        ]
    )

    # ========================================================================
    # DATA LOGGING
    # ========================================================================

    topic_data_logger = Node(
        package='scripts',
        executable='topic_data_logger',
        name='topic_data_logger',
        output='screen',
        parameters=[{
            'imu_topic': '/imu',
            'raw_odom_topic': '/wc_control/odom',
            'filtered_odom_topic': '/odometry/filtered',
            'log_frequency_hz': 10.0,
            'file_prefix': 'wheelchair_mapping_log'
        }]
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        declare_is_sim,
        declare_port,
        declare_model,
        declare_is_ignition,
        declare_use_sim_time,
        declare_world,
        declare_unite_imu,
        declare_lidar_port,
        declare_sudo_password,
        declare_rviz,
        declare_rviz_config,

        # USB permissions
        permission_setup,

        # Hardware + Sensors
        unified_wheelchair_launch,
        wheelchair_sensors_launch,
        rplidar_s3_launch,

        # Sensor Fusion
        static_transform_publisher,
        ekf_local_node,

        # SLAM
        slam_toolbox_mapping,
        map_saver_server,
        lifecycle_manager,

        # Logging
        topic_data_logger,
    ])
