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
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # IMU Filter (processes /imu/out to /imu/data_ekf)
    # This assumes your unified launch already provides /imu/out
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[
            {'use_mag': False},
            {'publish_tf': False},
            {'world_frame': 'enu'},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('imu/data_raw', '/imu/out'),
            ('imu/data', '/imu/data_ekf')
        ],
        output='screen'
    )

    # Path to the EKF configuration file
    ekf_config_path = os.path.join(
        get_package_share_directory('bumperbot_localization'),
        'config',
        'ekf.yaml'
    )

    # EKF Localization Node (delayed start to let IMU filter initialize)
    ekf_localization_node = TimerAction(
        period=3.0,  # Wait for IMU filter to start
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config_path,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('/odometry/filtered', '/odometry/filtered'),
                ]
            )
        ]
    )

    # Real-time visualization and data logger (delayed start)
    ekf_visualizer_node = TimerAction(
        period=5.0,  # Wait for everything to be ready
        actions=[
            Node(
                package='scripts',
                executable='ekf_realtime_visualizer.py',
                name='ekf_visualizer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_filter_node,
        ekf_localization_node,
        ekf_visualizer_node,
    ])