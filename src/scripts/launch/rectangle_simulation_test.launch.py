#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import conditions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='rectangle_waypoints.csv',
        description='Path to waypoint CSV file'
    )
    
    test_name_arg = DeclareLaunchArgument(
        'test_name',
        default_value='rectangle_sim_test',
        description='Name for the test (used in output files)'
    )
    
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.5',
        description='Maximum linear speed in m/s'
    )
    
    use_data_collector_arg = DeclareLaunchArgument(
        'use_data_collector',
        default_value='true',
        description='Whether to run data collector alongside navigator'
    )
    
    # Data collector node (optional)
    data_collector_node = Node(
        package='scripts',
        executable='data_collector',
        name='sim_data_collector',
        parameters=[{
            'use_sim_time': True,
            'output_file': [LaunchConfiguration('test_name'), '_full_data.csv']
        }],
        condition=conditions.IfCondition(LaunchConfiguration('use_data_collector')),
        output='screen'
    )
    
    # Rectangle navigation node (delayed start to allow system to initialize)
    rectangle_navigator_node = TimerAction(
        period=3.0,  # Wait 3 seconds before starting navigation
        actions=[
            Node(
                package='scripts',
                executable='rectangle_sim_navigator.py',
                name='rectangle_navigator',
                parameters=[{
                    'use_sim_time': True
                }],
                arguments=[
                    '--waypoints', LaunchConfiguration('waypoint_file'),
                    '--test-name', LaunchConfiguration('test_name'),
                    '--speed', LaunchConfiguration('speed')
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        waypoint_file_arg,
        test_name_arg,
        speed_arg,
        use_data_collector_arg,
        data_collector_node,
        rectangle_navigator_node,
    ])