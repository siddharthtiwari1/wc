#!/usr/bin/env python3

"""
Complete Loosely Coupled VIO System for Wheelchair Navigation
==============================================================

Launches complete Visual-Inertial-Encoder odometry system with three independent
odometry sources fused via Extended Kalman Filter.

System Architecture (Loosely Coupled):
├── Visual Odometry (RTAB-Map rgbd_odometry)
│   └── Publishes: /odom (x, y position + velocities)
├── Encoder Odometry (wc_control differential_drive_controller)
│   └── Publishes: /wc_control/odom (x, y position + x velocity)
├── IMU (RealSense D455)
│   └── Publishes: /imu (yaw orientation + angular velocity)
└── EKF Fusion (robot_localization)
    └── Publishes: /odometry/filtered (fused state estimate)

System Components:
1. Wheelchair hardware interface (encoders via ros2_control)
2. RealSense D455 Camera + IMU
3. IMU Filter (Madgwick) for orientation estimation
4. RTAB-Map Visual Odometry
5. EKF 3-way sensor fusion

Usage:
    ros2 launch wheelchair_bringup wheelchair_vio_complete.launch.py port:=/dev/ttyACM0

With visualization:
    ros2 launch wheelchair_bringup wheelchair_vio_complete.launch.py port:=/dev/ttyACM0 rviz:=true

Author: Siddharth Tiwari
Date: 2025-11-24
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Get package directories
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')
    wheelchair_localization_dir = get_package_share_directory('wheelchair_localization')
    wc_control_dir = get_package_share_directory('wc_control')

    # Default paths
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

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='True for simulation, false for real hardware.',
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for wheelchair hardware interface.',
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to the wheelchair URDF/xacro file.',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.',
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file (simulation only).',
    )

    declare_unite_imu = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='RealSense IMU synchronization (0=None, 1=copy, 2=linear interpolation).',
    )

    declare_wait_imu = DeclareLaunchArgument(
        'wait_imu_to_init',
        default_value='true',
        description='Block odometry until IMU orientation has initialized.',
    )

    declare_sudo_password = DeclareLaunchArgument(
        'sudo_password',
        default_value='12345',
        description='Sudo password for USB permissions.',
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

    declare_enable_plotting = DeclareLaunchArgument(
        'enable_plotting',
        default_value='true',
        description='Enable square/L-shape path EKF plotting.',
    )

    declare_test_type = DeclareLaunchArgument(
        'test_type',
        default_value='square_enhanced',
        description='Test type: square, square_enhanced, or lshape',
    )

    # Get configurations
    is_sim = LaunchConfiguration('is_sim')

    # ========================================================================
    # USB PERMISSIONS SETUP (hardware only)
    # ========================================================================

    def _grant_usb_permissions(context, *_args, **_kwargs):
        """Ensure serial devices are writable."""
        wheelchair_port = LaunchConfiguration('port').perform(context)
        password = LaunchConfiguration('sudo_password').perform(context)

        try:
            subprocess.run(
                ['sudo', '-S', 'chmod', '666', wheelchair_port],
                input=f'{password}\n',
                text=True,
                check=True,
            )
            print(f'✓ Updated permissions for {wheelchair_port}')
        except subprocess.CalledProcessError as exc:
            print(f'⚠ Failed to set permissions for {wheelchair_port}: {exc}')
        return []

    permission_setup = OpaqueFunction(
        function=_grant_usb_permissions,
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # WHEELCHAIR HARDWARE/CONTROL STACK
    # ========================================================================

    # Unified wheelchair launch (ros2_control, Gazebo, teleop, etc.)
    unified_wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wc_control_dir,
                'launch',
                'unified_wheelchair.launch.py',
            )
        ]),
        launch_arguments={
            'is_sim': is_sim,
            'port': LaunchConfiguration('port'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # ========================================================================
    # REALSENSE CAMERA + IMU
    # ========================================================================

    # Make sure IR emitter is enabled
    set_emitter = SetParameter(name='depth_module.emitter_enabled', value=1)

    # Launch RealSense camera with IMU
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
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # IMU FILTER (Madgwick)
    # ========================================================================

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
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # RTAB-MAP VISUAL ODOMETRY
    # ========================================================================

    rtabmap_parameters = [{
        'frame_id': 'camera_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
        'publish_tf': False,  # Let EKF publish TF
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
    ]

    rgbd_odometry = TimerAction(
        period=3.0,  # Wait for camera to initialize
        actions=[
            Node(
                package='rtabmap_odom',
                executable='rgbd_odometry',
                name='wheelchair_rgbd_odometry',
                output='screen',
                parameters=rtabmap_parameters,
                remappings=rtabmap_remappings,
                arguments=[LaunchConfiguration('odom_args')],
            )
        ],
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # EKF SENSOR FUSION (VO + Encoder + IMU)
    # ========================================================================

    # Static TF: base_link -> IMU frame
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu"]
    )

    # EKF Node: Complete VIO fusion
    ekf_vio_complete_node = TimerAction(
        period=6.0,  # Wait for all sensors to start
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_vio_complete_node',
                output='screen',
                parameters=[
                    os.path.join(
                        wheelchair_localization_dir,
                        'config',
                        'ekf_vio_complete.yaml',
                    ),
                    {
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ]
    )

    # ========================================================================
    # PATH TESTING/PLOTTING (SQUARE or L-SHAPE)
    # ========================================================================

    # Square path plotter (default)
    square_path_plotter = TimerAction(
        period=8.0,  # Wait for EKF to start
        actions=[
            Node(
                package='scripts',
                executable='square_path_ekf_tester',
                name='square_path_ekf_tester',
                output='screen',
                parameters=[{
                    'use_sim_time': is_sim,
                }],
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_plotting'), "' == 'true' and '",
            LaunchConfiguration('test_type'), "' == 'square'"
        ]))
    )

    # L-shape path plotter (for testing 6m forward + 90° turn + 4m forward)
    lshape_path_plotter = TimerAction(
        period=8.0,  # Wait for EKF to start
        actions=[
            Node(
                package='scripts',
                executable='l_shape_odometry_test.py',
                name='l_shape_odometry_test',
                output='screen',
                parameters=[{
                    'use_sim_time': is_sim,
                }],
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_plotting'), "' == 'true' and '",
            LaunchConfiguration('test_type'), "' == 'lshape'"
        ]))
    )

    # Enhanced Square path plotter with professional visualization
    square_enhanced_plotter = TimerAction(
        period=8.0,  # Wait for EKF to start
        actions=[
            Node(
                package='scripts',
                executable='square_odometry_test_enhanced.py',
                name='square_odometry_test_enhanced',
                output='screen',
                parameters=[{
                    'use_sim_time': is_sim,
                }],
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_plotting'), "' == 'true' and '",
            LaunchConfiguration('test_type'), "' == 'square_enhanced'"
        ]))
    )

    # ========================================================================
    # DATA LOGGER
    # ========================================================================

    topic_data_logger = Node(
        package='scripts',
        executable='topic_data_logger',
        name='vio_data_logger',
        output='screen',
        parameters=[{
            'imu_topic': '/imu',
            'raw_odom_topic': '/wc_control/odom',
            'filtered_odom_topic': '/odometry/filtered',
            'log_frequency_hz': 10.0,
            'file_prefix': 'wheelchair_localization_log'
        }]
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': is_sim}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        declare_is_sim,
        declare_port,
        declare_model,
        declare_use_sim_time,
        declare_world,
        declare_unite_imu,
        declare_wait_imu,
        declare_sudo_password,
        declare_rviz,
        declare_rviz_config,
        declare_odom_args,
        declare_enable_plotting,
        declare_test_type,

        # Permissions
        permission_setup,

        # Hardware/control stack
        unified_wheelchair_launch,

        # Sensor pipeline
        set_emitter,
        realsense_launch,
        imu_filter,
        rgbd_odometry,

        # Sensor fusion
        static_transform_imu,
        ekf_vio_complete_node,

        # Path testing and plotting
        square_path_plotter,
        lshape_path_plotter,
        square_enhanced_plotter,

        # Logging and visualization
        topic_data_logger,
        rviz_node,
    ])
