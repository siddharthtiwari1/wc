#!/usr/bin/env python3

"""
ALL-IN-ONE WHEELCHAIR SLAM MAPPING
===================================
Launches complete system for SLAM mapping (hardware + sensors + SLAM + RViz)

Usage:
    ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
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
    # Default RViz config - SLAM mapping
    default_rviz_config = os.path.join(
        wheelchair_description_dir,
        'rviz',
        'slam_mapping.rviz',
    )
    default_slam_config = os.path.join(
        wheelchair_localization_dir,
        'config',
        'slam_toolbox.yaml',
    )
    # Detect ROS distro for compatibility
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

    # Hardcoded values for real robot
    is_sim_value = 'false'
    port_value = '/dev/ttyACM0'
    lidar_port_value = '/dev/ttyUSB0'
    enable_plotting_value = 'false'  # No plotting during mapping

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value=is_sim_value,
        description='True for simulation, false for real hardware.',
    )
    declare_port = DeclareLaunchArgument(
        'port',
        default_value=port_value,
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
        default_value=lidar_port_value,
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
        default_value=enable_plotting_value,
        description='Enable square path EKF plotting.',
    )
    declare_test_type = DeclareLaunchArgument(
        'test_type',
        default_value='square',
        description='Test type: square or lshape',
    )
    declare_slam_config = DeclareLaunchArgument(
        'slam_config',
        default_value=default_slam_config,
        description='SLAM Toolbox configuration file.',
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

    # Hardware sensor pipeline (RealSense + IMU filtering/republisher)
    # NOTE: Disable RViz here since we launch our own with SLAM config
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
            'rviz': 'false',  # Disable RViz from sensors launch
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
        launch_arguments={'inverted': 'true'}.items(),  # CRITICAL FIX: S3 requires inverted=true
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

    # Bridge RPLidar frame (publishes as 'laser') to URDF frame ('lidar')
    # CRITICAL FIX: 180° rotation to correct backward scan orientation
    lidar_to_laser_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "1", "--qw", "0",
                   "--frame-id", "lidar",
                   "--child-frame-id", "laser"],
        condition=UnlessCondition(is_sim)
    )

    # Local EKF: Fuses wheels + IMU (fast, smooth, but drifts)
    ekf_local_node = TimerAction(
        period=3.0,  # Reduced delay for faster SLAM startup
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
    # SLAM TOOLBOX
    # ========================================================================

    # SLAM Toolbox lifecycle nodes list (ROS distro aware)
    slam_lifecycle_nodes = ["map_saver_server"]
    if ros_distro != "humble":
        slam_lifecycle_nodes.append("slam_toolbox")

    # SLAM nodes - no delays like bumperbot
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_config'),
            {'use_sim_time': is_sim}
        ],
        condition=UnlessCondition(is_sim)
    )

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[
            {'save_map_timeout': 5.0},
            {'use_sim_time': is_sim},
            {'free_thresh_default': 0.196},
            {'occupied_thresh_default': 0.65},
        ],
        condition=UnlessCondition(is_sim)
    )

    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'node_names': slam_lifecycle_nodes},
            {'use_sim_time': is_sim},
            {'autostart': True}
        ],
        condition=UnlessCondition(is_sim)
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================

    # RViz launches with a delay to ensure topics and transforms are available
    rviz_node = TimerAction(
        period=11.0,  # Wait for SLAM and all sensors to be ready
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', default_rviz_config],
                parameters=[{'use_sim_time': is_sim}],
                additional_env={'DISPLAY': ':1'},
            )
        ],
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
        declare_slam_config,

        # USB permissions (hardware only)
        permission_setup,

        # Unified motion stack + sensors
        unified_wheelchair_launch,
        wheelchair_sensors_launch,
        rplidar_s3_launch,

        # Localization
        static_transform_publisher,
        lidar_to_laser_transform,
        ekf_local_node,
        ekf_global_node,

        # SLAM
        slam_toolbox_node,
        map_saver_server,
        slam_lifecycle_manager,

        # Visualization and plotting
        square_path_plotter,
        lshape_path_plotter,
        topic_data_logger,
        rviz_node,
    ])
