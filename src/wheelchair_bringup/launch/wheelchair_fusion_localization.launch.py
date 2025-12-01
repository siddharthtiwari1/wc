#!/usr/bin/env python3

"""
WHEELCHAIR FUSION LOCALIZATION - VISUAL-LIDAR INTEGRATION
==========================================================
Publication-quality global localization with sensor fusion.

Launches:
    - Hardware interface (motors, ros2_control)
    - RPLidar S3 with robust filtering
    - RealSense D455 (depth + RGB + IMU)
    - EKF sensor fusion (wheel odom + IMU)
    - Enhanced AMCL (map-based localization)
    - Depth camera pointcloud processing
    - Map server

Key Features:
    - Robust laser filtering (6-stage chain)
    - Depth camera for frontal obstacle detection
    - High-precision AMCL (12k particles)
    - Real-time pose estimation

Usage:
    ros2 launch wheelchair_bringup wheelchair_fusion_localization.launch.py

    With custom map:
    ros2 launch wheelchair_bringup wheelchair_fusion_localization.launch.py \\
        map_name:=/path/to/map.yaml

Author: Siddharth Tiwari (s24035@students.iitmandi.ac.in)
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    GroupAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    # Get package directories
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')
    wheelchair_localization_dir = get_package_share_directory('wheelchair_localization')
    wheelchair_bringup_dir = get_package_share_directory('wheelchair_bringup')
    wc_control_dir = get_package_share_directory('wc_control')

    # Default paths
    default_model_path = os.path.join(
        wheelchair_description_dir, 'urdf', 'wheelchair_description.urdf.xacro'
    )
    default_rviz_config = os.path.join(
        wheelchair_description_dir, 'rviz', 'fusion_localization.rviz'
    )
    default_map_file = '/home/sidd/wc/maps/my_map.yaml'

    # Configuration files
    default_amcl_config = os.path.join(
        wheelchair_localization_dir, 'config', 'amcl_fusion.yaml'
    )
    default_ekf_config = os.path.join(
        wheelchair_localization_dir, 'config', 'ekf.yaml'
    )
    default_laser_filter_config = os.path.join(
        wheelchair_localization_dir, 'config', 'laser_filter_robust.yaml'
    )

    # ROS distro detection
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

    # Hardware defaults
    is_sim_value = 'false'
    port_value = '/dev/ttyACM0'
    lidar_port_value = '/dev/ttyUSB0'

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value=default_map_file,
        description='Full path to map YAML file.'
    )
    declare_is_sim = DeclareLaunchArgument(
        'is_sim', default_value=is_sim_value,
        description='True for simulation, false for real hardware.'
    )
    declare_port = DeclareLaunchArgument(
        'port', default_value=port_value,
        description='Serial port for wheelchair hardware interface.'
    )
    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Absolute path to the wheelchair URDF/xacro file.'
    )
    declare_is_ignition = DeclareLaunchArgument(
        'is_ignition', default_value=default_is_ignition,
        description='Set to true when running on Humble/ros_ign combination.'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time if true.'
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Gazebo world file (simulation only).'
    )
    declare_unite_imu = DeclareLaunchArgument(
        'unite_imu_method', default_value='2',
        description='RealSense IMU synchronization (0=None, 1=copy, 2=linear interpolation).'
    )
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value=lidar_port_value,
        description='Serial port for RPLidar (hardware only).'
    )
    declare_sudo_password = DeclareLaunchArgument(
        'sudo_password', default_value='12345',
        description='Sudo password for USB permissions (hardware only).'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz for visualization.'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='RViz configuration file.'
    )
    declare_amcl_config = DeclareLaunchArgument(
        'amcl_config', default_value=default_amcl_config,
        description='Full path to AMCL yaml configuration file.'
    )
    declare_use_depth = DeclareLaunchArgument(
        'use_depth', default_value='true',
        description='Enable depth camera for obstacle detection.'
    )

    # Get launch configurations
    is_sim = LaunchConfiguration('is_sim')
    map_name = LaunchConfiguration('map_name')
    amcl_config = LaunchConfiguration('amcl_config')
    use_rviz = LaunchConfiguration('rviz')
    use_depth = LaunchConfiguration('use_depth')

    # ========================================================================
    # USB PERMISSIONS SETUP
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
            print(f'[INFO] Skipping permission change for {lidar_port}: device not present')

        for port in ports:
            try:
                subprocess.run(
                    ['sudo', '-S', 'chmod', '666', port],
                    input=f'{password}\n',
                    text=True,
                    check=True,
                    capture_output=True,
                )
                print(f'[OK] Updated permissions for {port}')
            except subprocess.CalledProcessError as exc:
                print(f'[WARN] Failed to set permissions for {port}: {exc}')
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
        PythonLaunchDescriptionSource(
            os.path.join(wc_control_dir, 'launch', 'unified_wheelchair.launch.py')
        ),
        launch_arguments={
            'is_sim': LaunchConfiguration('is_sim'),
            'port': LaunchConfiguration('port'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Hardware sensor pipeline (RealSense + IMU filtering)
    wheelchair_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wc_control_dir, 'launch', 'wheelchair_sensors.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'is_ignition': LaunchConfiguration('is_ignition'),
            'is_sim': LaunchConfiguration('is_sim'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'rviz': 'false',
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
        condition=UnlessCondition(is_sim)
    )

    # RPLidar S3
    rplidar_s3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch', 'rplidar_s3_launch.py'
            )
        ),
        launch_arguments={'inverted': 'true'}.items(),
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # ROBUST LASER FILTER (6-stage chain)
    # ========================================================================

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[default_laser_filter_config],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ],
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # TF TRANSFORMS
    # ========================================================================

    # Static TF: base_link -> imu
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu']
    )

    # Static TF: lidar -> laser (180° rotation for correct orientation)
    lidar_to_laser_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '1', '--qw', '0',
                   '--frame-id', 'lidar', '--child-frame-id', 'laser'],
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # EKF LOCALIZATION (wheel odom + IMU)
    # ========================================================================

    ekf_local_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    default_ekf_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ]
            )
        ]
    )

    # ========================================================================
    # GLOBAL LOCALIZATION - MAP SERVER + AMCL
    # ========================================================================

    lifecycle_nodes = ['map_server', 'amcl']

    # Map Server
    nav2_map_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': map_name},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            )
        ]
    )

    # AMCL
    nav2_amcl = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                emulate_tty=True,
                parameters=[
                    amcl_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            )
        ]
    )

    # Lifecycle Manager for localization
    nav2_lifecycle_manager = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'node_names': lifecycle_nodes},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                ],
            )
        ]
    )

    # ========================================================================
    # VELOCITY COMMAND BRIDGE
    # ========================================================================

    twist_to_stamped_converter = Node(
        package='scripts',
        executable='twist_stamped_teleop',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
            'file_prefix': 'wheelchair_fusion_log'
        }]
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================

    rviz_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                condition=IfCondition(use_rviz),
            )
        ],
    )

    # ========================================================================
    # STARTUP MESSAGES
    # ========================================================================

    startup_message = TimerAction(
        period=13.0,
        actions=[
            LogInfo(msg='=' * 60),
            LogInfo(msg='  WHEELCHAIR FUSION LOCALIZATION READY'),
            LogInfo(msg='=' * 60),
            LogInfo(msg='  Sensors: RPLidar S3 + RealSense D455'),
            LogInfo(msg='  Filter:  6-stage robust laser filter'),
            LogInfo(msg='  AMCL:    12k particles, 300 beams'),
            LogInfo(msg='=' * 60),
        ]
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
        declare_use_depth,

        # USB permissions (hardware only)
        permission_setup,

        # Hardware and sensors
        unified_wheelchair_launch,
        wheelchair_sensors_launch,
        rplidar_s3_launch,

        # Laser filtering
        laser_filter_node,

        # TF transforms
        static_transform_imu,
        lidar_to_laser_transform,

        # EKF localization
        ekf_local_node,

        # Global localization (AMCL)
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,

        # Velocity command bridge
        twist_to_stamped_converter,

        # Logging and visualization
        topic_data_logger,
        rviz_node,
        startup_message,
    ])
