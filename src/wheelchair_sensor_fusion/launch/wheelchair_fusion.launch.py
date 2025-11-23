#!/usr/bin/env python3
"""
Wheelchair Sensor Fusion Launch File
=====================================

Launches production-ready sensor fusion system integrated with wheelchair
navigation stack for indoor obstacle avoidance.

Features:
- Robust adaptive sensor fusion (LiDAR + Camera + YOLO)
- Automatic sensor health monitoring and fallback
- Nav2 costmap integration
- Real-time performance (30 Hz)
- Indoor environment optimization

Author: Siddharth Tiwari
Institution: Indian Institute of Technology Mandi
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for wheelchair sensor fusion."""

    # Package directories
    fusion_pkg = FindPackageShare('wheelchair_sensor_fusion')

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',                    # RViz already launched by wheelchair system
        description='Launch RViz visualization'
    )

    yolo_model = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov11n.pt',              # Nano model for real-time
        description='YOLO model: yolov11n/s/m/l/x.pt'
    )

    device = DeclareLaunchArgument(
        'device',
        default_value='cuda',                      # GPU acceleration
        description='YOLO device: cuda or cpu'
    )

    confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO confidence threshold'
    )

    enable_tracking = DeclareLaunchArgument(
        'enable_tracking',
        default_value='true',
        description='Enable multi-frame obstacle tracking'
    )

    indoor_mode = DeclareLaunchArgument(
        'indoor_mode',
        default_value='true',                      # Wheelchair is indoor-focused
        description='Enable indoor-specific optimizations'
    )

    # ========================================================================
    # CONFIGURATION PATHS
    # ========================================================================

    config_dir = PathJoinSubstitution([fusion_pkg, 'config'])
    wheelchair_config = PathJoinSubstitution([config_dir, 'wheelchair_integration.yaml'])
    rviz_config = PathJoinSubstitution([fusion_pkg, 'rviz', 'sensor_fusion.rviz'])

    # ========================================================================
    # NODES
    # ========================================================================

    # LiDAR Processor - DBSCAN clustering
    lidar_processor = Node(
        package='wheelchair_sensor_fusion',
        executable='lidar_processor_node',
        name='lidar_processor',
        output='screen',
        parameters=[wheelchair_config],
        remappings=[
            ('/scan', '/scan'),                    # RPLidar S3 topic from wheelchair
        ]
    )

    # YOLO Detector - Object detection
    yolo_detector = Node(
        package='wheelchair_sensor_fusion',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[
            wheelchair_config,
            {
                'model_path': LaunchConfiguration('yolo_model'),
                'device': LaunchConfiguration('device'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            }
        ],
        remappings=[
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/color/camera_info', '/camera/color/camera_info'),
        ]
    )

    # Robust Sensor Fusion - Adaptive fusion with fault tolerance
    sensor_fusion = Node(
        package='wheelchair_sensor_fusion',
        executable='sensor_fusion_node_robust',
        name='sensor_fusion_robust',
        output='screen',
        parameters=[
            wheelchair_config,
            {
                'enable_tracking': LaunchConfiguration('enable_tracking'),
                'indoor_mode': LaunchConfiguration('indoor_mode'),
            }
        ],
        remappings=[
            ('/camera/aligned_depth_to_color/image_raw', '/camera/aligned_depth_to_color/image_raw'),
        ]
    )

    # Obstacle Publisher - Nav2 costmap integration
    obstacle_publisher = Node(
        package='wheelchair_sensor_fusion',
        executable='obstacle_publisher_node',
        name='obstacle_publisher',
        output='screen',
        parameters=[wheelchair_config]
    )

    # Optional: RViz visualization (usually already running from wheelchair bringup)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fusion',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        use_rviz,
        yolo_model,
        device,
        confidence_threshold,
        enable_tracking,
        indoor_mode,

        # Nodes
        lidar_processor,
        yolo_detector,
        sensor_fusion,
        obstacle_publisher,
        rviz,
    ])
