#!/usr/bin/env python3

"""
Bumperbot/Wheelchair Mapping and Localization System
=====================================================

Comprehensive launch file for creating maps and performing robust localization.

Two Main Modes:
---------------
1. MAPPING MODE (mode:=mapping)
   - Creates a new map using slam_toolbox
   - Saves map for future localization
   - Run this first to explore and map your environment

2. LOCALIZATION MODE (mode:=localization)
   - Uses AMCL for robust Monte Carlo localization
   - Loads pre-built map
   - Provides accurate global pose for navigation
   - Integrates with Nav2 for autonomous navigation

System Architecture:
-------------------
1. Hardware/Control Layer:
   - Wheelchair hardware interface + ros2_control
   - Differential drive controllers
   - RPLidar S3 laser scanner

2. Sensor Fusion:
   - Local EKF: Fuses wheels + IMU → /odometry/filtered (smooth, local)
   - Global pose from SLAM/AMCL → /map frame

3. Mapping/Localization:
   - MAPPING: slam_toolbox (async mode for real-time performance)
   - LOCALIZATION: AMCL (robust particle filter localization)

Usage Examples:
--------------
# 1. Create a map (drive around to explore):
ros2 launch wheelchair_bringup bumperbot_mapping_localization.launch.py \
    mode:=mapping \
    is_sim:=false \
    port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0

# 2. Save the map when done:
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_environment

# 3. Localize on existing map:
ros2 launch wheelchair_bringup bumperbot_mapping_localization.launch.py \
    mode:=localization \
    map_file:=/home/user/maps/my_environment.yaml \
    is_sim:=false \
    port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0

# 4. With Nav2 for autonomous navigation:
ros2 launch wheelchair_bringup bumperbot_mapping_localization.launch.py \
    mode:=localization \
    map_file:=/home/user/maps/my_environment.yaml \
    enable_nav2:=true

"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # Detect ROS distro for compatibility
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    # Core arguments
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Operation mode: "mapping" to create map, "localization" for AMCL on existing map',
        choices=['mapping', 'localization']
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map yaml file (required for localization mode)',
    )

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
        default_value='/dev/ttyUSB0',
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

    declare_enable_nav2 = DeclareLaunchArgument(
        'enable_nav2',
        default_value='false',
        description='Enable Nav2 navigation stack (localization mode only)',
    )

    # Get launch configurations
    is_sim = LaunchConfiguration('is_sim')
    mode = LaunchConfiguration('mode')

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
    # HARDWARE & CONTROL STACK
    # ========================================================================

    # Unified wheelchair hardware/control stack (ros2_control, Gazebo, teleop, sim helpers)
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

    # Hardware sensor pipeline (RealSense + IMU filtering/republisher + RViz)
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
    # LOCALIZATION - LOCAL EKF (ALWAYS RUNNING)
    # ========================================================================

    # Static transform for IMU
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu"]
    )

    # Local EKF: Fuses wheels + IMU (fast, smooth, but drifts)
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
                        get_package_share_directory('wheelchair_localization'),
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
    # MAPPING MODE - SLAM TOOLBOX
    # ========================================================================

    # SLAM Toolbox for mapping (async for real-time performance)
    slam_toolbox_mapping = TimerAction(
        period=8.0,  # Wait for lidar and EKF to start
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
                        'slam_toolbox_mapping.yaml',
                    ),
                    {
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'mapping'"
        ]))
    )

    # ========================================================================
    # LOCALIZATION MODE - MAP SERVER + AMCL
    # ========================================================================

    # Map server for pre-built maps
    map_server_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {
                        'yaml_filename': LaunchConfiguration('map_file'),
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'localization'"
        ]))
    )

    # Lifecycle manager for map server
    map_server_lifecycle = TimerAction(
        period=9.0,
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
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'localization'"
        ]))
    )

    # AMCL for robust localization
    amcl_node = TimerAction(
        period=10.0,  # Wait for map server to load
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    os.path.join(
                        wheelchair_localization_dir,
                        'config',
                        'amcl.yaml',
                    ),
                    {
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'localization'"
        ]))
    )

    # Lifecycle manager for AMCL
    amcl_lifecycle = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_amcl',
                output='screen',
                parameters=[
                    {
                        'node_names': ['amcl'],
                        'autostart': True,
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'localization'"
        ]))
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
            'file_prefix': 'bumperbot_data_log'
        }]
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================

    rviz_condition = PythonExpression([
        "'",
        LaunchConfiguration('rviz'),
        "' == 'true'"
    ])

    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')],
                parameters=[{'use_sim_time': is_sim}],
                condition=IfCondition(rviz_condition),
            )
        ]
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        declare_mode,
        declare_map_file,
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
        declare_enable_nav2,

        # USB permissions (hardware only)
        permission_setup,

        # Unified motion stack + sensors
        unified_wheelchair_launch,
        wheelchair_sensors_launch,
        rplidar_s3_launch,

        # Localization - Local EKF (always running)
        static_transform_publisher,
        ekf_local_node,

        # MAPPING MODE
        slam_toolbox_mapping,

        # LOCALIZATION MODE
        map_server_node,
        map_server_lifecycle,
        amcl_node,
        amcl_lifecycle,

        # Visualization and logging
        topic_data_logger,
        rviz_node,
    ])
