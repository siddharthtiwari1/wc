import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'unite_imu_method',
            default_value='2',
            description=(
                '0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" '
                'if IMU topics stop being published.'
            ),
        ),

        SetParameter(name='depth_module.emitter_enabled', value=1),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py',
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

        Node(
            package='wc_control',
            executable='imu_base_link_republisher.py',
            name='imu_base_link_republisher',
            output='screen',
            parameters=[{
                'input_topic': '/imu/data',
                'output_topic': '/imu',
                'output_frame': 'base_link',
                'base_to_optical_quaternion': [-0.5, 0.5, -0.5, 0.5],
            }],
        ),
    ])
