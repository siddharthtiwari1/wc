#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    🚁 COMPREHENSIVE WHEELCHAIR SYSTEM LAUNCH

    Includes ALL functionality:
    ✅ Hardware Interface (Arduino + ros2_control)
    ✅ RealSense Camera + IMU
    ✅ Sensor Fusion (EKF + Custom Kalman)
    ✅ RTAB-Map SLAM + Visual Odometry
    ✅ Control System
    ✅ LiDAR Support
    ✅ Navigation Stack
    ✅ Visualization (RViz)

    Professional staged startup sequence for stability.
    """

    # ==================== PACKAGE DIRECTORIES ====================
    pkg_wheelchair_bringup = get_package_share_directory('wheelchair_bringup')
    pkg_wheelchair_firmware = get_package_share_directory('wheelchair_firmware')
    pkg_wheelchair_localization = get_package_share_directory('wheelchair_localization')
    pkg_wheelchair_description = get_package_share_directory('wheelchair_description')
    pkg_wc_control = get_package_share_directory('wc_control')

    # ==================== LAUNCH ARGUMENTS ====================
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'is_sim', default_value='false',
            description='Whether running in simulation'
        ),
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyACM0',
            description='Serial port for Arduino'
        ),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='Serial port for LiDAR'
        ),
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation for RealSense IMU'
        ),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Enable RTAB-Map SLAM'
        ),
        DeclareLaunchArgument(
            'use_navigation', default_value='true',
            description='Enable navigation stack'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz visualization'
        ),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Enable LiDAR'
        ),
        DeclareLaunchArgument(
            'use_localization', default_value='true',
            description='Enable comprehensive localization system'
        ),
        DeclareLaunchArgument(
            'camera_rgb_profile', default_value='640x360x30',
            description='RealSense RGB camera profile'
        ),
        DeclareLaunchArgument(
            'camera_depth_profile', default_value='640x360x30',
            description='RealSense depth camera profile'
        )
    ]

    # ==================== LAUNCH CONFIGURATIONS ====================
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_sim = LaunchConfiguration('is_sim')
    port = LaunchConfiguration('port')
    lidar_port = LaunchConfiguration('lidar_port')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    use_slam = LaunchConfiguration('use_slam')
    use_navigation = LaunchConfiguration('use_navigation')
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')
    use_localization = LaunchConfiguration('use_localization')
    camera_rgb_profile = LaunchConfiguration('camera_rgb_profile')
    camera_depth_profile = LaunchConfiguration('camera_depth_profile')

    # ==================== STAGE 1: CORE HARDWARE (Start Immediately) ====================

    # Modern ros2_control Hardware Interface
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheelchair_firmware'),
                'launch',
                'ros2_control_hardware_interface.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'is_sim': is_sim,
            'port': port
        }.items(),
        condition=UnlessCondition(is_sim)
    )

    # LiDAR Driver
    lidar_driver = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'lidar',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(PythonExpression([use_lidar, ' and not ', is_sim]))
    )

    # ==================== STAGE 2: CAMERA SYSTEM (3 seconds delay) ====================

    # RealSense Camera with comprehensive configuration
    realsense_camera = GroupAction([
        # Enable IR emitter for better depth quality
        SetParameter(name='depth_module.emitter_enabled', value=1),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'camera_namespace': '',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': unite_imu_method,
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                'rgb_camera.profile': camera_rgb_profile,
                'depth_module.profile': camera_depth_profile,
                'enable_pointcloud': 'true',
                'pointcloud.allow_no_texture_points': 'true',
                'use_sim_time': use_sim_time
            }.items()
        )
    ], condition=UnlessCondition(is_sim))

    # Camera IMU Processing - Madgwick Filter
    camera_imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='camera_imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_sim_time': use_sim_time,
            'stateless': False,
            'constant_dt': 0.0,
            'publish_debug_topics': True
        }],
        remappings=[
            ('imu/data_raw', '/camera/imu'),
            ('imu/data', '/imu/data')
        ],
        condition=UnlessCondition(is_sim)
    )

    # IMU Republisher for sensor fusion
    imu_republisher = Node(
        package='wheelchair_firmware',
        executable='imu_republisher.py',
        name='imu_republisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('imu_in', '/imu/data'),
            ('imu_out', '/imu/out')
        ],
        condition=UnlessCondition(is_sim)
    )

    # ==================== STAGE 3: LOCALIZATION SYSTEM (6 seconds delay) ====================

    # Comprehensive Localization System (EKF + Custom Kalman)
    localization_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheelchair_localization'),
                'launch',
                'localization.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_localization)
    )

    # ==================== STAGE 4: SLAM SYSTEM (9 seconds delay) ====================

    # RTAB-Map parameters optimized for wheelchair
    rtabmap_parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_scan': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': True,
        'use_sim_time': use_sim_time,
        # Wheelchair-optimized RTAB-Map parameters
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        'Grid/FromDepth': 'false',  # Use laser scan for occupancy
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Reg/Strategy': '1',  # Visual + ICP registration
        'Vis/EstimationType': '1',  # 3D->2D (PnP)
        'Vis/MaxDepth': '4.0',
        'Optimizer/GravitySigma': '0.3',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Kp/MaxFeatures': '400',
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1'
    }]

    rtabmap_remappings = [
        ('imu', '/imu/out'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('scan', '/scan'),
        ('odom', '/odometry/filtered')  # Use EKF filtered odometry
    ]

    # RTAB-Map Visual Odometry
    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        condition=IfCondition(use_slam)
    )

    # RTAB-Map SLAM
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        arguments=['-d'],  # Delete database on startup
        condition=IfCondition(use_slam)
    )

    # RTAB-Map Visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        condition=IfCondition(PythonExpression([use_slam, ' and ', use_rviz]))
    )

    # ==================== STAGE 5: CONTROL SYSTEM (12 seconds delay) ====================

    # Control System
    control_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wc_control'),
                'launch',
                'wheelchair_controller.py'
            ])
        ),
        launch_arguments={
            'use_simple_controller': 'False',
            'use_python': 'False',
            'use_sim_time': use_sim_time
        }.items()
    )

    # ==================== STAGE 6: VISUALIZATION (15 seconds delay) ====================

    # RViz Visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('wheelchair_description'),
        'rviz',
        'urdf_config.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([use_rviz, ' and not ', use_slam]))
    )

    # ==================== PROFESSIONAL STAGED STARTUP ====================

    # Stage 1: Core Hardware (immediate start)
    stage1_hardware = GroupAction([
        hardware_interface,
        lidar_driver
    ])

    # Stage 2: Camera System (3 seconds)
    stage2_camera = TimerAction(
        period=3.0,
        actions=[
            realsense_camera,
            camera_imu_filter,
            imu_republisher
        ]
    )

    # Stage 3: Localization System (6 seconds)
    stage3_localization = TimerAction(
        period=6.0,
        actions=[localization_system]
    )

    # Stage 4: SLAM System (9 seconds)
    stage4_slam = TimerAction(
        period=9.0,
        actions=[
            rtabmap_odom,
            rtabmap_slam
        ]
    )

    # Stage 5: Control System (12 seconds)
    stage5_control = TimerAction(
        period=12.0,
        actions=[control_system]
    )

    # Stage 6: Visualization (15 seconds)
    stage6_visualization = TimerAction(
        period=15.0,
        actions=[
            rtabmap_viz,
            rviz
        ]
    )

    # ==================== SYSTEM STATUS MONITOR ====================

    # System Status Monitor
    status_monitor = Node(
        package='wheelchair_bringup',
        executable='system_status_monitor.py',
        name='system_status_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitor_topics': [
                '/joint_states',
                '/camera/color/image_raw',
                '/scan',
                '/odometry/filtered',
                '/wc_control/cmd_vel'
            ]
        }]
    )

    # ==================== SAFETY MONITOR ====================

    # Safety Monitor
    safety_monitor = Node(
        package='wheelchair_bringup',
        executable='safety_monitor.py',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_velocity': 2.68,
            'max_angular_velocity': 17.59,
            'emergency_stop_topic': '/emergency_stop'
        }]
    )

    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # Environment setup
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Launch arguments
        *declare_args,

        # Staged startup sequence
        stage1_hardware,       # Immediate: Hardware + LiDAR
        stage2_camera,         # 3s: Camera + IMU processing
        stage3_localization,   # 6s: EKF + Kalman filter
        stage4_slam,          # 9s: RTAB-Map SLAM
        stage5_control,       # 12s: Control system
        stage6_visualization, # 15s: Visualization

        # System monitoring
        TimerAction(period=18.0, actions=[status_monitor]),
        TimerAction(period=20.0, actions=[safety_monitor]),
    ])