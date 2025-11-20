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

    # Optimized EKF Node - directly specify all parameters inline
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_optimized',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'frequency': 50.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'transform_time_offset': 0.0,
                'transform_timeout': 0.0,
                'print_diagnostics': True,
                'debug': False,
                'publish_tf': True,
                'publish_acceleration': False,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'odom0': '/wc_control/odom',
                'imu0': '/imu/odom_oriented',
                'odom0_config': [True, True, False, False, False, True, True, True, False, False, False, False, False, False, False],
                'imu0_config': [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False],
                'imu0_remove_gravitational_acceleration': False,
                'odom0_differential': False,
                'odom0_queue_size': 10,
                # CONSERVATIVE COVARIANCES - key fix!
                'odom0_pose_covariance': [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
                                         0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.5],  # Higher wheel yaw uncertainty
                'odom0_twist_covariance': [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
                                          0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
                'imu0_pose_covariance': [1e6, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1],  # Trust IMU yaw moderately
                'imu0_twist_covariance': [1e6, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.05],  # Trust IMU yaw rate
                # VERY CONSERVATIVE PROCESS NOISE
                'process_noise_covariance': [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
            }
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered_ultra_conservative'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_localization_node,
    ])