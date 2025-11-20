#!/usr/bin/env python3

"""Launch all wheelchair sensors: RealSense camera, IMU filtering, and RPLidar S3."""

import os
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')
    default_model_path = os.path.join(
        wheelchair_description_dir,
        'urdf',
        'wheelchair_description.urdf.xacro',
    )
    default_rviz_config = os.path.join(
        wheelchair_description_dir,
        'rviz',
        'urdf_config.rviz',
    )

    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to the wheelchair URDF/xacro describing camera and IMU frames.',
    )
    declare_is_ignition = DeclareLaunchArgument(
        'is_ignition',
        default_value=default_is_ignition,
        description='Set to true when running on Humble/ros_ign combination so Gazebo bindings work.',
    )
    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='True when running simulation interfaces (passed to the wheelchair xacro).',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.',
    )
    declare_unite_imu = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='RealSense IMU synchronization strategy (0=None, 1=copy, 2=linear interpolation).',
    )
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port used by the RPLidar.',
    )
    declare_lidar_baudrate = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='Serial baudrate for the RPLidar.',
    )
    declare_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar',
        description='TF frame for laser scans.',
    )
    declare_sudo_password = DeclareLaunchArgument(
        'sudo_password',
        default_value='12345',
        description='Password used to update USB permissions before starting sensors.',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization.',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file.',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('model'),
            ' is_ignition:=',
            LaunchConfiguration('is_ignition'),
            ' is_sim:=',
            LaunchConfiguration('is_sim'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    joint_state_actions = []
    joint_state_publisher = None
    try:
        get_package_share_directory('joint_state_publisher_gui')
        joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )
    except PackageNotFoundError:
        try:
            get_package_share_directory('joint_state_publisher')
            joint_state_actions.append(LogInfo(msg='joint_state_publisher_gui not found; using joint_state_publisher.'))
            joint_state_publisher = Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
            )
        except PackageNotFoundError:
            joint_state_actions.append(LogInfo(msg='No joint state publisher package found; continuing without joint publisher.'))

    def _grant_usb_permissions(context, *_args, **_kwargs):
        """Ensure the LiDAR serial device is writable before nodes start."""
        port = LaunchConfiguration('lidar_port').perform(context)
        password = LaunchConfiguration('sudo_password').perform(context)
        try:
            subprocess.run(
                ['sudo', '-S', 'chmod', '666', port],
                input=f'{password}\n',
                text=True,
                check=True,
            )
            print(f'✓ Updated permissions for {port}')
        except subprocess.CalledProcessError as exc:
            print(f'⚠ Failed to set permissions for {port}: {exc}')
        return []

    permission_setup = OpaqueFunction(function=_grant_usb_permissions)

    realsense_launch = IncludeLaunchDescription(
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
            'depth_module.emitter_enabled': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'rgb_camera.profile': '640x360x30',
            'depth_module.profile': '640x360x30',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('imu/data_raw', '/camera/imu')],
    )

    imu_wheelchair_republisher = Node(
        package='wc_control',
        executable='imu_wheelchair_republisher.py',
        name='imu_wheelchair_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/imu/data',
            'output_topic': '/imu',
            'output_frame': 'imu',
            'orientation_quaternion': [0.5, -0.5, 0.5, 0.5],
            'vector_quaternion': [0.5, 0.5, 0.5, 0.5],
            'zero_on_start': True,
        }],
    )

    # RPLidar S3 - simple inclusion of working launch file
    rplidar_s3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s3_launch.py',
            )
        ]),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_model,
        declare_is_ignition,
        declare_use_sim_time,
        declare_is_sim,
        declare_unite_imu,
        declare_lidar_port,
        declare_lidar_baudrate,
        declare_lidar_frame,
        declare_sudo_password,
        declare_rviz,
        declare_rviz_config,
        permission_setup,
        robot_state_publisher,
        *joint_state_actions,
        *( [joint_state_publisher] if joint_state_publisher else [] ),
        realsense_launch,
        imu_filter,
        imu_wheelchair_republisher,
        rplidar_s3_launch,
        rviz_node,
    ])
