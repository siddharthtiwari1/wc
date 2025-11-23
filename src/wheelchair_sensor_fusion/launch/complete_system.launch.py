#!/usr/bin/env python3
"""Complete wheelchair sensor fusion system launch file.

Launches all sensors + fusion + navigation.

Author: Siddharth Tiwari
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate complete system launch description."""

    # Package shares
    fusion_pkg = FindPackageShare('wheelchair_sensor_fusion')
    wc_control_pkg = FindPackageShare('wc_control')

    # Launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Include sensor launch (RealSense + RPLidar + IMU)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                wc_control_pkg,
                'wc_control',
                'all_sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'false',  # We'll launch RViz from fusion
        }.items()
    )

    # Include sensor fusion launch
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fusion_pkg,
                'launch',
                'sensor_fusion.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )

    return LaunchDescription([
        use_rviz,
        sensors_launch,
        fusion_launch,
    ])
