#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    
    # Launch configuration variables
    is_sim = LaunchConfiguration('is_sim')
    port = LaunchConfiguration('port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    collect_data = LaunchConfiguration('collect_data')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'is_sim',
            default_value='false',
            description='Use simulation or real hardware'
        ),
        
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino communication'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'collect_data',
            default_value='true',
            description='Enable data collection for analysis'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization'
        ),

        # 1. Wheelchair Control System (unified launch)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('wc_control'),
                    'launch',
                    'unified_wheelchair.launch.py'
                ])
            ]),
            launch_arguments={
                'is_sim': is_sim,
                'port': port
            }.items(),
        ),

        # 2. RealSense D435i Camera with IMU (delayed start to allow hardware to initialize)
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            get_package_share_directory('rtabmap_examples'),
                            'launch',
                            'realsense_d435i_color.launch.py'
                        ])
                    ]),
                )
            ]
        ),

        # 3. Robot Localization (EKF) - delayed start to allow odometry to stabilize
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            get_package_share_directory('wheelchair_localization'),
                            'launch',
                            'localization_with_viz.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items(),
                )
            ]
        ),

        # 4. Data Collection Node (delayed start to ensure all topics are available)
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='scripts',
                    executable='data_collector',
                    name='wheelchair_data_collector',
                    output='screen',
                    condition=IfCondition(collect_data),
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'log_frequency': 10.0
                    }]
                )
            ]
        ),

        # 5. Safety and System Monitors
        Node(
            package='wheelchair_bringup',
            executable='safety_monitor.py',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='wheelchair_bringup', 
            executable='system_status_monitor.py',
            name='system_status_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # 6. Additional RViz for complete system visualization (delayed start)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_complete',
                    output='screen',
                    condition=IfCondition(use_rviz),
                    arguments=['-d', PathJoinSubstitution([
                        get_package_share_directory('wheelchair_description'),
                        'rviz',
                        'urdf_config.rviz'
                    ])],
                    parameters=[{
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        ),

    ])