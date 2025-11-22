#!/usr/bin/env python3
"""
RAN Complete System Launch File

Launches all components for wheelchair attribute-aware navigation:
1. Hardware (sensors, localization)
2. Perception (YOLO + SAM2 + DINOv2 + CLIP)
3. Mapping (adaptive clustering + attribute extraction)
4. Navigation (hierarchical verification + Nav2)
5. Safety monitoring

Author: Siddharth Tiwari
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file', default='')

    # Package shares
    ran_complete_share = FindPackageShare('ran_complete')
    wheelchair_bringup_share = FindPackageShare('wheelchair_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    # Params file
    ran_params_file = PathJoinSubstitution([
        ran_complete_share, 'config', 'ran_params.yaml'
    ])

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Path to pre-built semantic map'
        ),

        # 1. Wheelchair hardware (sensors + localization)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    wheelchair_bringup_share,
                    'launch',
                    'wheelchair_hardware.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 2. RAN Perception Node
        Node(
            package='ran_complete',
            executable='ran_perception_node.py',
            name='ran_perception',
            parameters=[ran_params_file, {'use_sim_time': use_sim_time}],
            output='screen',
            emulate_tty=True
        ),

        # 3. RAN Mapping Node
        Node(
            package='ran_complete',
            executable='ran_mapping_node.py',
            name='ran_mapping',
            parameters=[ran_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 4. RAN Hierarchical Verifier (NOVEL!)
        Node(
            package='ran_complete',
            executable='ran_hierarchical_verifier.py',
            name='ran_verifier',
            parameters=[ran_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 5. RAN Navigator
        Node(
            package='ran_complete',
            executable='ran_navigator.py',
            name='ran_navigator',
            parameters=[ran_params_file, {
                'use_sim_time': use_sim_time,
                'map_file': map_file
            }],
            output='screen'
        ),

        # 6. Safety Monitor (wheelchair-specific)
        Node(
            package='ran_complete',
            executable='ran_safety_monitor.py',
            name='ran_safety',
            parameters=[ran_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 7. Nav2 (for path planning)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    nav2_bringup_share,
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': ran_params_file
            }.items()
        ),

        # 8. RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                ran_complete_share, 'rviz', 'ran_wheelchair.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
