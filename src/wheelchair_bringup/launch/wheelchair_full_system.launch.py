#!/usr/bin/env python3

"""
Comprehensive Wheelchair System Bringup
========================================

Launches complete wheelchair system in one command:
1. Wheelchair hardware interface + ros2_control + controllers (motors + wheel odometry)
2. All sensors (RealSense camera + IMU, RPLidar S3)
3. EKF sensor fusion localization
4. Professional odometry visualization (square/L-shape path testing)
5. RViz visualization

Usage:
    Real hardware with enhanced square visualization (DEFAULT):
        ros2 launch wheelchair_bringup wheelchair_full_system.launch.py is_sim:=false port:=/dev/ttyACM0

    With basic square test:
        ros2 launch wheelchair_bringup wheelchair_full_system.launch.py is_sim:=false port:=/dev/ttyACM0 test_type:=square

    With L-shape test:
        ros2 launch wheelchair_bringup wheelchair_full_system.launch.py is_sim:=false port:=/dev/ttyACM0 test_type:=lshape

    Disable plotting:
        ros2 launch wheelchair_bringup wheelchair_full_system.launch.py is_sim:=false port:=/dev/ttyACM0 enable_plotting:=false

    Simulation:
        ros2 launch wheelchair_bringup wheelchair_full_system.launch.py is_sim:=true
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    # Default RViz config
    default_rviz_config = os.path.join(
        wheelchair_description_dir,
        'rviz',
        'urdf_config.rviz',
    )
    # Detect ROS distro for compatibility
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

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
    declare_is_ignition = DeclareLaunchArgument(
        'is_ignition',
        default_value=default_is_ignition,
        description='Set to true when running on Humble/ros_ign combination.',
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
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar (hardware only).',
    )
    declare_sudo_password = DeclareLaunchArgument(
        'sudo_password',
        default_value='12345',
        description='Sudo password for USB permissions (hardware only).',
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
    declare_enable_plotting = DeclareLaunchArgument(
        'enable_plotting',
        default_value='true',
        description='Enable square path EKF plotting.',
    )
    declare_test_type = DeclareLaunchArgument(
        'test_type',
        default_value='square',
        description='Test type: square, square_enhanced, or lshape',
    )

    # Get launch configurations
    is_sim = LaunchConfiguration('is_sim')

    # ========================================================================
    # USB PERMISSIONS SETUP (only for real hardware)
    # ========================================================================

    def _grant_usb_permissions(context, *_args, **_kwargs):
        """Ensure serial devices are writable before nodes start."""
        lidar_port = LaunchConfiguration('lidar_port').perform(context)
        wheelchair_port = LaunchConfiguration('port').perform(context)
        password = LaunchConfiguration('sudo_password').perform(context)

        ports = [wheelchair_port]
        if os.path.exists(lidar_port):
            ports.append(lidar_port)
        else:
            print(f'ℹ Skipping permission change for {lidar_port}: device not present')

        for port in ports:
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

    permission_setup = OpaqueFunction(
        function=_grant_usb_permissions,
        condition=UnlessCondition(is_sim)
    )

    # Unified wheelchair hardware/control stack (ros2_control, Gazebo, teleop, sim helpers)
    unified_wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wc_control_dir,
                'launch',
                'unified_wheelchair.launch.py',
            )
        ]),
        launch_arguments={
            'is_sim': LaunchConfiguration('is_sim'),
            'port': LaunchConfiguration('port'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Hardware sensor pipeline (RealSense + IMU filtering/republisher + RViz)
    wheelchair_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                wc_control_dir,
                'launch',
                'wheelchair_sensors.launch.py',
            )
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'is_ignition': LaunchConfiguration('is_ignition'),
            'is_sim': LaunchConfiguration('is_sim'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
        condition=UnlessCondition(is_sim)
    )

    # RPLidar S3
    rplidar_s3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s3_launch.py',
            )
        ]),
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # EKF LOCALIZATION (both simulation and hardware)
    # ========================================================================
    # NOTE: SLAM mapping is now in separate launch file: wheelchair_slam.launch.py

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu"]
    )

    # Local EKF: Fuses wheels + IMU (fast, smooth, but drifts)
    ekf_local_node = TimerAction(
        period=6.0,  # Wait for sensors and controllers to start
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    os.path.join(
                        get_package_share_directory('wheelchair_localization'),
                        'config',
                        'ekf.yaml',
                    ),
                    {
                        'use_sim_time': is_sim,
                    }
                ]
            )
        ]
    )

    # Global EKF: Fuses local odometry + SLAM (drift-free global pose)
    ekf_global_node = TimerAction(
        period=10.0,  # Wait for SLAM to initialize
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global_node',
                output='screen',
                parameters=[
                    os.path.join(
                        get_package_share_directory('wheelchair_localization'),
                        'config',
                        'ekf_global.yaml',
                    ),
                    {
                        'use_sim_time': is_sim,
                    }
                ],
                remappings=[
                    ('/odometry/filtered', '/odometry/global')
                ]
            )
        ],
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # PATH TESTING/PLOTTING (SQUARE or L-SHAPE)
    # ========================================================================

    # Square path plotter (default)
    square_path_plotter = TimerAction(
        period=7.0,  # Wait for EKF to start
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
        period=7.0,  # Wait for EKF to start
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
        period=7.0,  # Wait for EKF to start
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
        name='topic_data_logger',
        output='screen',
        parameters=[{
            'imu_topic': '/imu',
            'raw_odom_topic': '/wc_control/odom',
            'filtered_odom_topic': '/odometry/filtered',
            'log_frequency_hz': 10.0,
            'file_prefix': 'wheelchair_data_log'
        }]
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================

    # RViz launches when rviz:=true (works for both sim and hardware)
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
        # Launch arguments
        declare_is_sim,
        declare_port,
        declare_model,
        declare_is_ignition,
        declare_use_sim_time,
        declare_world,
        declare_unite_imu,
        declare_lidar_port,
        declare_sudo_password,
        declare_rviz,
        declare_rviz_config,
        declare_enable_plotting,
        declare_test_type,

        # USB permissions (hardware only)
        permission_setup,

        # Unified motion stack + sensors
        unified_wheelchair_launch,
        wheelchair_sensors_launch,
        rplidar_s3_launch,

        # Localization
        static_transform_publisher,
        ekf_local_node,
        ekf_global_node,

        # Visualization and plotting
        square_path_plotter,
        lshape_path_plotter,
        square_enhanced_plotter,
        topic_data_logger,
        rviz_node,
    ])
