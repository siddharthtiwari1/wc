#!/usr/bin/env python3

"""
Wheelchair Visual-Inertial Odometry System (VO + IMU Loosely Coupled)
======================================================================

Launches visual-inertial odometry using RTAB-Map VO and IMU fusion via EKF.

This is a loosely coupled system where:
- RTAB-Map rgbd_odometry provides visual odometry (/odom)
- IMU provides orientation (/imu)
- robot_localization EKF fuses both sources

System Components:
1. RealSense D455 Camera + IMU
2. IMU Filter (Madgwick) for orientation estimation
3. RTAB-Map Visual Odometry (rgbd_odometry)
4. EKF Sensor Fusion (robot_localization)

Usage:
    ros2 launch wheelchair_bringup wheelchair_vo_imu_fusion.launch.py

Author: Siddharth Tiwari
Date: 2025-11-24
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Get package directories
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')
    wheelchair_localization_dir = get_package_share_directory('wheelchair_localization')

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

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to the wheelchair URDF/xacro file.',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.',
    )

    declare_unite_imu = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='RealSense IMU synchronization (0=None, 1=copy, 2=linear interpolation).',
    )

    declare_wait_imu = DeclareLaunchArgument(
        'wait_imu_to_init',
        default_value='true',
        description='Block odometry until IMU orientation has initialized.',
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

    declare_odom_args = DeclareLaunchArgument(
        'odom_args',
        default_value='',
        description='Extra CLI arguments forwarded to rgbd_odometry.',
    )

    # ========================================================================
    # REALSENSE CAMERA + IMU
    # ========================================================================

    # Make sure IR emitter is enabled for depth
    set_emitter = SetParameter(name='depth_module.emitter_enabled', value=1)

    # Launch RealSense camera with IMU
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ]),
        launch_arguments={
            'camera_namespace': '',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'depth_module.emitter_enabled': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'rgb_camera.profile': '640x360x30',
            'depth_module.profile': '640x360x30',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # ========================================================================
    # IMU FILTER (Madgwick)
    # ========================================================================

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('imu/data_raw', '/camera/imu')],
    )

    # ========================================================================
    # RTAB-MAP VISUAL ODOMETRY
    # ========================================================================

    rtabmap_parameters = [{
        'frame_id': 'camera_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
        'publish_tf': False,  # Let EKF publish TF
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
    ]

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='wheelchair_rgbd_odometry',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        arguments=[LaunchConfiguration('odom_args')],
    )

    # ========================================================================
    # EKF SENSOR FUSION (VO + IMU)
    # ========================================================================

    # Static TF: base_link -> IMU frame
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu"]
    )

    # EKF Node: Fuses VO + IMU
    ekf_vo_imu_node = TimerAction(
        period=5.0,  # Wait for sensors to initialize
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_vo_imu_node',
                output='screen',
                parameters=[
                    os.path.join(
                        wheelchair_localization_dir,
                        'config',
                        'ekf_vo_imu.yaml',
                    ),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ]
            )
        ]
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        declare_model,
        declare_use_sim_time,
        declare_unite_imu,
        declare_wait_imu,
        declare_rviz,
        declare_rviz_config,
        declare_odom_args,

        # Sensor pipeline
        set_emitter,
        realsense_launch,
        imu_filter,
        rgbd_odometry,

        # Sensor fusion
        static_transform_imu,
        ekf_vo_imu_node,

        # Visualization
        rviz_node,
    ])
