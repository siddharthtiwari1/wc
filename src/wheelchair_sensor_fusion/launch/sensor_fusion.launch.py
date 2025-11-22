#!/usr/bin/env python3
"""Launch file for complete sensor fusion system.

Launches:
  - LiDAR processor node
  - YOLO detector node
  - Sensor fusion node
  - Obstacle publisher node
  - RViz visualization

Author: Siddharth Tiwari
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
    """Generate launch description for sensor fusion system."""

    # Package directories
    fusion_pkg = FindPackageShare('wheelchair_sensor_fusion')

    # Launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    yolo_model = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov11n.pt',
        description='YOLO model file (n/s/m/l/x)'
    )

    device = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device for YOLO inference (cuda/cpu)'
    )

    confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO confidence threshold'
    )

    # Config file paths
    config_dir = PathJoinSubstitution([fusion_pkg, 'config'])
    rviz_config = PathJoinSubstitution([fusion_pkg, 'rviz', 'sensor_fusion.rviz'])

    # Nodes
    lidar_processor = Node(
        package='wheelchair_sensor_fusion',
        executable='lidar_processor_node',
        name='lidar_processor',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'lidar_processor.yaml'])
        ]
    )

    yolo_detector = Node(
        package='wheelchair_sensor_fusion',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('yolo_model'),
            'device': LaunchConfiguration('device'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
        }]
    )

    sensor_fusion = Node(
        package='wheelchair_sensor_fusion',
        executable='sensor_fusion_node',
        name='sensor_fusion',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'sensor_fusion.yaml'])
        ]
    )

    obstacle_publisher = Node(
        package='wheelchair_sensor_fusion',
        executable='obstacle_publisher_node',
        name='obstacle_publisher',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'obstacle_publisher.yaml'])
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz,
        yolo_model,
        device,
        confidence_threshold,
        lidar_processor,
        yolo_detector,
        sensor_fusion,
        obstacle_publisher,
        rviz,
    ])
