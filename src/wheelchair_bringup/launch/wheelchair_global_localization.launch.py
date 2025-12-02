#!/usr/bin/env python3

"""
ALL-IN-ONE WHEELCHAIR GLOBAL LOCALIZATION (AMCL)
=================================================
Complete system for global localization using pre-built map.
Based on bumperbot_localization and wheelchair_slam_mapping.launch.py

Launches:
    ✓ Hardware interface (motors, controllers)
    ✓ RPLidar S3 driver (/scan topic)
    ✓ RealSense D455 camera + IMU
    ✓ EKF sensor fusion → /odometry/filtered
    ✓ Map server (loads your saved map)
    ✓ AMCL (global localization with particle filter)
    ✓ RViz (visualization)

Usage:
    Real hardware:
        ros2 launch wheelchair_bringup wheelchair_global_localization.launch.py

    Custom map:
        ros2 launch wheelchair_bringup wheelchair_global_localization.launch.py map_name:=/home/sidd/maps/my_map.yaml
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
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
    # Default RViz config for localization
    default_rviz_config = os.path.join(
        wheelchair_description_dir,
        'rviz',
        'global_localization.rviz',
    )
    # Default map file
    default_map_file = '/home/sidd/wc/maps/my_map.yaml'

    # Detect ROS distro for compatibility
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

    # Hardcoded values for real robot
    is_sim_value = 'false'
    port_value = '/dev/ttyACM0'
    lidar_port_value = '/dev/ttyUSB0'

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value=default_map_file,
        description='Full path to map YAML file.',
    )

    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value=is_sim_value,
        description='True for simulation, false for real hardware.',
    )
    declare_port = DeclareLaunchArgument(
        'port',
        default_value=port_value,
        description='Serial port for wheelchair hardware interface.',
    )
    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to the wheelchair URDF/xacro file.',
    )
    declare_is_ignition = DeclareLaunchArgument(
        'is_ignition',
        default_value=default_is_ignition,
        description='Set to true when running on Humble/ros_ign combination.',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file (simulation only).',
    )
    declare_unite_imu = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='RealSense IMU synchronization (0=None, 1=copy, 2=linear interpolation).',
    )
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value=lidar_port_value,
        description='Serial port for RPLidar (hardware only).',
    )
    declare_sudo_password = DeclareLaunchArgument(
        'sudo_password',
        default_value='12345',
        description='Sudo password for USB permissions (hardware only).',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization.',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file.',
    )
    declare_amcl_config = DeclareLaunchArgument(
        'amcl_config',
        default_value=os.path.join(
            wheelchair_localization_dir,
            'config',
            'amcl_v2.yaml'
        ),
        description='Full path to AMCL yaml configuration file',
    )

    # Get launch configurations
    is_sim = LaunchConfiguration('is_sim')
    map_name = LaunchConfiguration('map_name')
    amcl_config = LaunchConfiguration('amcl_config')
    use_rviz = LaunchConfiguration('rviz')

    # ========================================================================
    # USB PERMISSIONS SETUP (only for real hardware)
    # ========================================================================

    def _grant_usb_permissions(context, *_args, **_kwargs):
        """Ensure serial devices are writable before nodes start."""
        lidar_port = LaunchConfiguration('lidar_port').perform(context)
        wheelchair_port = LaunchConfiguration('port').perform(context)
        password = LaunchConfiguration('sudo_password').perform(context)

        ports = [wheelchair_port]
        if os.path.exists(lidar_port):
            ports.append(lidar_port)
        else:
            print(f'ℹ Skipping permission change for {lidar_port}: device not present')

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

    # Unified wheelchair hardware/control stack
    unified_wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wc_control_dir,
                'launch',
                'unified_wheelchair.launch.py',
            )
        ]),
        launch_arguments={
            'is_sim': LaunchConfiguration('is_sim'),
            'port': LaunchConfiguration('port'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Hardware sensor pipeline (RealSense + IMU filtering/republisher)
    wheelchair_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wc_control_dir,
                'launch',
                'wheelchair_sensors.launch.py',
            )
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'is_ignition': LaunchConfiguration('is_ignition'),
            'is_sim': LaunchConfiguration('is_sim'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'rviz': 'false',  # We'll launch our own RViz with localization config
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
        launch_arguments={'inverted': 'true'}.items(),  # S3 requires inverted=true
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # SENSOR FUSION - LOCAL EKF
    # ========================================================================

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu"]
    )

    # Bridge RPLidar frame (publishes as 'laser') to URDF frame ('lidar')
    # CRITICAL FIX: 180° rotation to correct backward scan orientation
    lidar_to_laser_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "1", "--qw", "0",
                   "--frame-id", "lidar",
                   "--child-frame-id", "laser"],
        condition=UnlessCondition(is_sim)
    )

    # Local EKF: Fuses wheels + IMU (fast, smooth, provides /odometry/filtered)
    ekf_local_node = TimerAction(
        period=6.0,  # Wait for sensors and controllers to start
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    os.path.join(
                        wheelchair_localization_dir,
                        'config',
                        'ekf.yaml',
                    ),
                    {
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ]
    )

    # ========================================================================
    # GLOBAL LOCALIZATION - MAP SERVER + AMCL
    # ========================================================================

    # Lifecycle nodes for lifecycle manager
    lifecycle_nodes = ["map_server", "amcl"]

    # Nav2 Map Server node
    nav2_map_server = TimerAction(
        period=8.0,  # Wait for EKF to initialize
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"yaml_filename": map_name},
                    {"use_sim_time": is_sim}
                ],
            )
        ]
    )

    # Nav2 AMCL node
    nav2_amcl = TimerAction(
        period=9.0,  # Wait for map server
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                emulate_tty=True,
                parameters=[
                    amcl_config,
                    {"use_sim_time": is_sim},
                ],
            )
        ]
    )

    # Nav2 Lifecycle Manager
    nav2_lifecycle_manager = TimerAction(
        period=10.0,  # Wait for both nodes to start
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"node_names": lifecycle_nodes},
                    {"use_sim_time": is_sim},
                    {"autostart": True}
                ],
            )
        ]
    )

    # ========================================================================
    # VELOCITY COMMAND BRIDGE (CRITICAL FOR MOTION)
    # ========================================================================
    # diff_drive_controller expects TwistStamped on /wc_control/cmd_vel
    # Nav2 and teleop publish Twist on /cmd_vel
    # This bridge converts Twist → TwistStamped with proper timestamps
    # Benefits: Stale command detection, latency compensation, safety timeouts

    twist_to_stamped_converter = Node(
        package='scripts',
        executable='twist_stamped_teleop',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': is_sim}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/wc_control/cmd_vel'),
        ],
    )

    # ========================================================================
    # DATA LOGGER
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
            'file_prefix': 'wheelchair_localization_log'
        }]
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================

    # RViz visualization - launches immediately using ExecuteProcess
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', default_rviz_config],
        output='screen',
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        declare_map_name,
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
        declare_amcl_config,

        # USB permissions (hardware only)
        permission_setup,

        # Unified motion stack + sensors
        unified_wheelchair_launch,
        wheelchair_sensors_launch,
        rplidar_s3_launch,

        # Localization
        static_transform_publisher,
        lidar_to_laser_transform,
        ekf_local_node,

        # Global localization (AMCL)
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,

        # Velocity command bridge (enables /cmd_vel → controller)
        twist_to_stamped_converter,

        # Visualization and logging
        topic_data_logger,
        rviz_node,
    ])
