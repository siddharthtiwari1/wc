#!/usr/bin/env python3

"""Launch RealSense camera, IMU filtering, and frame-aligned republishers (no RTAB-Map odom)."""

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
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization.',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file.',
    )
    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='If true, launch robot_state_publisher. Set false when included from another launch.',
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

    # Only launch robot_state_publisher in standalone mode
    # When included from wheelchair_full_system.launch.py, unified_wheelchair provides this
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('standalone')),
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
            condition=IfCondition(LaunchConfiguration('standalone')),
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
                condition=IfCondition(LaunchConfiguration('standalone')),
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

    # ========================================================================
    # IMU PROCESSING PIPELINE
    # ========================================================================
    # 0. imu_startup_calibrator: Measures gyro bias at startup (3 seconds)
    #    Publishes to /imu/calibrated_bias for dynamic bias update
    # 1. imu_bias_corrector: Applies gyro bias correction to raw IMU
    #    /camera/imu -> /camera/imu_corrected
    #    (subscribes to /imu/calibrated_bias for dynamic updates)
    # 2. imu_filter_madgwick: Fuses accel+gyro into orientation
    #    /camera/imu_corrected -> /imu/data
    # 3. imu_wheelchair_republisher: Transforms to base_link frame
    #    /imu/data -> /imu
    # ========================================================================

    # Step 0: Startup calibration - measures current gyro bias
    # IMPORTANT: Wheelchair must be stationary for first 3 seconds after boot
    imu_startup_calibrator = Node(
        package='wc_control',
        executable='imu_startup_calibrator.py',
        name='imu_startup_calibrator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/camera/imu',
            'bias_topic': '/imu/calibrated_bias',
            'status_topic': '/imu/calibration_status',
            'calibration_duration': 3.0,  # seconds
            'min_samples': 100,
            'max_motion_threshold': 0.05,  # rad/s
            # Default bias values (fallback if calibration fails)
            'default_gyro_x_bias': -0.004302,
            'default_gyro_y_bias': 0.000787,
            'default_gyro_z_bias': 0.000948,
        }],
    )

    # Step 1: Apply gyro bias correction BEFORE Madgwick filter
    # Starts with default static calibration values, updates when startup calibration completes
    imu_bias_corrector = Node(
        package='wc_control',
        executable='imu_bias_corrector.py',
        name='imu_bias_corrector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/camera/imu',
            'output_topic': '/camera/imu_corrected',
            'calibrated_bias_topic': '/imu/calibrated_bias',
            # Default gyro bias values in camera frame (rad/s)
            # From static test 2025-12-06 17:38 (full_system_20251206_173829.csv)
            # Will be overridden by startup calibrator if successful
            'gyro_x_bias': -0.004302,
            'gyro_y_bias': 0.000787,
            'gyro_z_bias': 0.000948,
        }],
    )

    # Step 2: Madgwick filter - now reads from corrected topic
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
        remappings=[('imu/data_raw', '/camera/imu_corrected')],
    )

    # Step 3: Transform to base_link frame + apply gyro Z bias in base_link frame
    # Using hardcoded quaternions (from original github launch file) - NOT TF lookup
    # orientation_quaternion: [0.5, -0.5, 0.5, 0.5] - for orientation transform
    # vector_quaternion: [0.5, 0.5, 0.5, 0.5] - for angular velocity/linear acceleration
    imu_wheelchair_republisher = Node(
        package='wc_control',
        executable='imu_wheelchair_republisher.py',
        name='imu_wheelchair_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/imu/data',
            'output_topic': '/imu',
            'output_frame': 'base_link',
            'zero_on_start': True,
            # Use TF to stay in sync with robot_description; fallback quaternion is still available
            'use_tf': True,
            'orientation_quaternion': [0.5, -0.5, 0.5, 0.5],
            # Gyro Z bias in BASE_LINK frame (measured from 2025-12-06 static test)
            # Applied in bias corrector before Madgwick; set to zero here to avoid double-biasing
            'gyro_z_bias': 0.0,  # rad/s - will be subtracted from measurement
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
        declare_rviz,
        declare_rviz_config,
        declare_standalone,
        robot_state_publisher,
        *joint_state_actions,
        *( [joint_state_publisher] if joint_state_publisher else [] ),
        realsense_launch,
        imu_startup_calibrator,  # Must start before bias corrector to publish bias
        imu_bias_corrector,
        imu_filter,
        imu_wheelchair_republisher,
        rviz_node,
    ])
