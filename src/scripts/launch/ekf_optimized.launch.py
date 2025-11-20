#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # IMU Filter (processes /imu/out to /imu/odom_oriented)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[
            {'use_mag': False},
            {'publish_tf': False},
            {'world_frame': 'enu'},
            {'use_sim_time': use_sim_time},
            {'gain': 0.1},  # Lower gain for stability
            {'zeta': 0.0},  # No gyro bias estimation
        ],
        remappings=[
            ('imu/data_raw', '/imu/out'),
            ('imu/data', '/imu/odom_oriented')
        ],
        output='screen'
    )

    # Optimized EKF configuration path
    ekf_config_path = os.path.join(
        get_package_share_directory('wheelchair_localization'),
        'config',
        'ekf_optimized.yaml'
    )

    # EKF Localization Node with optimized parameters
    ekf_localization_node = TimerAction(
        period=2.0,  # Wait for IMU filter to start
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node_optimized',
                output='screen',
                parameters=[
                    ekf_config_path,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('/odometry/filtered', '/odometry/filtered_optimized'),
                ]
            )
        ]
    )

    # Comparison tool with optimized filtered data
    comparison_node = TimerAction(
        period=4.0,  # Wait for EKF to start
        actions=[
            Node(
                package='scripts',
                executable='ekf_odom_comparison',
                name='ekf_optimized_comparison',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'filtered_topic': '/odometry/filtered_optimized'}
                ]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_filter_node,
        ekf_localization_node,
        comparison_node,
    ])