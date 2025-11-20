#!/usr/bin/env python3

"""
Wheelchair Tightly-Coupled Visual-Inertial Odometry (VIO) + SLAM Launch File

This launch file provides:
- RealSense D435i camera with IMU
- IMU filtering (Madgwick) for orientation estimation
- Tightly-coupled VIO using RTAB-Map with IMU constraints
- SLAM mapping mode for environment mapping
- IMU and odometry republishers for frame transformations
- Robot state publisher for URDF/TF
- RViz visualization

Key Features:
- Tightly-coupled VIO prevents drift during rotation
- IMU gravity constraints improve odometry accuracy
- Full SLAM capability with loop closure
- Frame transformations from camera to base_link
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ParameterValue as PV


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

    # ========== Launch Arguments ==========
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
    declare_rtabmap_viz = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map native visualization.',
    )
    declare_delete_db = DeclareLaunchArgument(
        'delete_db',
        default_value='false',
        description='Delete existing RTAB-Map database on start.',
    )
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Set to true for localization mode (vs mapping mode).',
    )
    declare_odom_args = DeclareLaunchArgument(
        'odom_args',
        default_value='',
        description='Extra CLI arguments forwarded to rgbd_odometry.',
    )

    # ========== Robot Description ==========
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

    # ========== Joint State Publisher ==========
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

    # ========== RealSense Camera + IMU ==========
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

    # ========== IMU Filter (Madgwick) ==========
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

    # ========== RTAB-Map Odometry Parameters ==========
    rtabmap_odom_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
        'publish_tf': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),

        # VIO: Use IMU to constrain visual odometry during rotation
        'Odom/Strategy': '0',  # 0=Frame-to-Map (uses local map)
        'Odom/AlignWithGround': 'true',  # Use IMU gravity for ground plane
        'Optimizer/GravitySigma': '0.3',  # IMU gravity constraint (lower = stronger)

        # When rotating: Trust IMU rotation, constrain VO translation
        'Odom/GuessMotion': 'true',  # Use IMU motion as initial guess
        'OdomF2M/BundleAdjustment': '1',  # Optimize with IMU constraints

        # Reject bad visual matches during rotation
        'Vis/MinInliers': '20',  # Require more inliers (stricter)
        'Vis/InlierDistance': '0.05',  # Max reprojection error (stricter)
    }]

    # ========== RTAB-Map SLAM Parameters ==========
    rtabmap_slam_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'publish_tf': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),

        # Mapping mode
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data'),  # Use raw IMU from Madgwick filter, RTAB-Map will use TF
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
    ]

    # ========== RTAB-Map Visual-Inertial Odometry Node ==========
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='wheelchair_rgbd_odometry',
        output='screen',
        parameters=rtabmap_odom_parameters,
        remappings=rtabmap_remappings,
        arguments=[LaunchConfiguration('odom_args')],
    )

    # ========== RTAB-Map SLAM Node (Mapping) ==========
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='wheelchair_rtabmap_slam',
        output='screen',
        parameters=rtabmap_slam_parameters,
        remappings=rtabmap_remappings,
        arguments=['--delete_db_on_start'],
    )

    # ========== RTAB-Map Visualization (Optional) ==========
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=rtabmap_slam_parameters,
        remappings=rtabmap_remappings,
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    # ========== Odometry Republisher (odom → base_link frame) ==========
    odom_camera_republisher = Node(
        package='wc_control',
        executable='odom_camera_republisher.py',
        name='odom_camera_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/odom',
            'output_topic': '/odom/camera',
        }],
    )

    # ========== IMU Republisher (for other nodes, not used by RTAB-Map) ==========
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
            'sensor_to_base_quaternion': [0.5, -0.5, 0.5, 0.5],  # camera_imu_optical_frame → base_link
        }],
    )

    # ========== RViz Visualization ==========
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # Launch arguments
        declare_model,
        declare_is_ignition,
        declare_use_sim_time,
        declare_is_sim,
        declare_unite_imu,
        declare_wait_imu,
        declare_rviz,
        declare_rviz_config,
        declare_rtabmap_viz,
        declare_delete_db,
        declare_localization,
        declare_odom_args,

        # Robot description
        robot_state_publisher,
        *joint_state_actions,
        *([joint_state_publisher] if joint_state_publisher else []),

        # Sensors
        realsense_launch,
        imu_filter,

        # RTAB-Map VIO + SLAM
        rgbd_odometry,
        rtabmap_slam,
        rtabmap_viz_node,

        # Frame republishers
        odom_camera_republisher,
        imu_wheelchair_republisher,

        # Visualization
        rviz_node,
    ])
