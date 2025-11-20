#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
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
    declare_wait_imu = DeclareLaunchArgument(
        'wait_imu_to_init',
        default_value='true',
        description='Block odometry until IMU orientation has initialized.',
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
    declare_odom_args = DeclareLaunchArgument(
        'odom_args',
        default_value='',
        description='Extra CLI arguments forwarded to rgbd_odometry.',
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

    # Joint state publisher handling mirrors wheelchair_description/urdf_launch
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

    rtabmap_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
        'publish_tf': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
    ]

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='wheelchair_rgbd_odometry',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        arguments=[LaunchConfiguration('odom_args')],
    )

    odom_camera_republisher = Node(
        package='wc_control',
        executable='odom_camera_republisher.py',
        name='odom_camera_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/odom',
            'output_topic': '/odom/camera',
            'camera_frame': 'camera_link',
            'odom_to_base_translation': [0.0, 0.0, 0.0],
            'odom_to_base_quaternion': [0.005, 0.023, -0.707, 0.707],
        }],
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
            'sensor_to_base_quaternion': [0.5, -0.5, 0.5, 0.5],
        }],
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
        declare_wait_imu,
        declare_rviz,
        declare_rviz_config,
        declare_odom_args,
        robot_state_publisher,
        *joint_state_actions,
        *( [joint_state_publisher] if joint_state_publisher else [] ),
        realsense_launch,
        imu_filter,
        rgbd_odometry,
        odom_camera_republisher,
        imu_wheelchair_republisher,
        rviz_node,
    ])
