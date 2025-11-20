"""
RealSense D435i + RTABMAP SLAM with RViz Visualization
This launch file combines RealSense camera, IMU filtering, and RTABMAP with both RTABMAP Viz and RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'unite_imu_method',
            default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation'
        ),

        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='true',
            description='Launch RTABMAP native visualization'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz visualization'
        ),

        DeclareLaunchArgument(
            'delete_db',
            default_value='false',
            description='Delete existing RTABMAP database on start'
        ),

        # Enable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch RealSense D435i camera with IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch', 'rs_launch.py'
                )
            ]),
            launch_arguments={
                'camera_namespace': '',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                'rgb_camera.profile': '640x360x30',
            }.items(),
        ),

        # IMU Madgwick Filter - computes orientation from IMU
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
            }],
            remappings=[('imu/data_raw', '/camera/imu')],
        ),

        # Launch RTABMAP with proper RealSense topics
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('wc_control'),
                    'launch', 'rtabmap.launch.py'
                )
            ]),
            launch_arguments={
                # Frame IDs
                'frame_id': 'camera_link',
                'map_frame_id': 'map',

                # RealSense Topics
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'imu_topic': '/imu/data',

                # Synchronization
                'approx_sync': 'false',
                'wait_imu_to_init': 'true',

                # Odometry
                'visual_odometry': 'true',
                'icp_odometry': 'false',
                'publish_tf_odom': 'true',

                # Visualization
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),

                # RTABMAP arguments
                'args': '--delete_db_on_start' if LaunchConfiguration('delete_db').perform(None) == 'true' else '',

                # QoS
                'qos': '1',  # Reliable
            }.items(),
        ),
    ])
