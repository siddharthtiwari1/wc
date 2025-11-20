#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'wheelchair_localization'

    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time for EKF'
    )

    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='IMU topic to fuse (use /imu/out in simulation)'
    )
    
    # Path to corrected EKF configuration
    ekf_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ekf.yaml'  # Use corrected and consolidated config
    )
    
    # EKF Localization Node  
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {
                'use_sim_time': use_sim_time,
                'imu0': imu_topic
            }
        ]
    )
    
    # Static transform to guarantee EKF can transform IMU data into base_link
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu"]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_imu_topic,
        ekf_localization_node,
        static_transform_publisher,
    ])
