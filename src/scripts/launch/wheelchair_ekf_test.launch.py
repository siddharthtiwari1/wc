#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    is_sim_arg = DeclareLaunchArgument(
        'is_sim', 
        default_value='true',
        description='Use simulation mode'
    )
    
    is_sim = LaunchConfiguration('is_sim')

    # Include unified wheelchair launch
    unified_wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('wc_control'), 'launch', 'unified_wheelchair.launch.py')
        ]),
        launch_arguments={'is_sim': is_sim}.items()
    )

    # No longer needed - IMU orientation transformer handles frame conversion

    # Path to the EKF configuration file
    ekf_config_path = os.path.join(
        get_package_share_directory('wheelchair_localization'),
        'config',
        'ekf.yaml'
    )

    # EKF Localization Node (delayed start)
    ekf_localization_node = TimerAction(
        period=3.0,  # Wait for IMU transformer to start
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config_path,
                    {'use_sim_time': is_sim}
                ],
                remappings=[
                    ('/odometry/filtered', '/odometry/filtered'),
                ]
            )
        ]
    )

    # Real-time visualization and data logger (delayed start)
    ekf_visualizer_node = TimerAction(
        period=7.0,  # Wait for everything to be ready
        actions=[
            Node(
                package='scripts',
                executable='ekf_realtime_visualizer.py',
                name='ekf_visualizer',
                output='screen',
                parameters=[{'use_sim_time': is_sim}]
            )
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        unified_wheelchair_launch,
        ekf_localization_node,
        ekf_visualizer_node,
    ])