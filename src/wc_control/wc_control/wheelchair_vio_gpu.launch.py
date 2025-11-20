#!/usr/bin/env python3

"""
Wheelchair GPU-ACCELERATED Visual-Inertial Odometry (VIO) + SLAM Launch File

GPU OPTIMIZATIONS:
- NVIDIA RTX GPU acceleration (CUDA 12+)
- GPU-accelerated feature extraction (ORB-CUDA, FAST-GPU)
- GPU-accelerated feature matching (BruteForce-GPU)
- Maximum IMU utilization with tightly-coupled VIO
- D435i/D455 specific depth range and resolution tuning
- Enhanced bundle adjustment with larger local map
- Optimized for rotation stability

Hardware Requirements:
- RealSense D435i or D455 with IMU
- NVIDIA GPU with CUDA 12+ (RTX 3050/4050/5050 or better)
- ROS2 Jazzy/Humble with RTAB-Map built with CUDA support
- OpenCV built with CUDA support

Performance Targets:
- Rotation: <1cm drift during 360° rotation
- Translation: <3% error over 1m movement
- Update Rate: 30-60 Hz odometry (2-3x faster than CPU)
"""

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
        default_value='true',
        description='Delete existing RTAB-Map database on start.',
    )
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Set to true for localization mode (vs mapping mode).',
    )
    declare_enable_gpu = DeclareLaunchArgument(
        'enable_gpu',
        default_value='true',
        description='Enable GPU acceleration (requires OpenCV with CUDA).',
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

    # ========== RTAB-Map OPTIMIZED VIO Odometry Parameters ==========
    rtabmap_odom_parameters = [{
        # Frame configuration
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
        'publish_tf': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),

        # ========== MAXIMUM IMU UTILIZATION ==========
        # IMU gravity constraints (STRONGEST setting for rotation stability)
        'Odom/AlignWithGround': 'true',
        'Optimizer/GravitySigma': '0.2',  # Lower = stronger constraint (0.1-0.3 range)
        'Mem/UseOdomGravity': 'true',  # Use odometry gravity for loop closure

        # IMU motion prediction
        'Odom/GuessMotion': 'true',  # Use IMU for initial motion estimate
        'Odom/GuessSmoothingDelay': '0.1',  # Smooth velocity over 100ms

        # ========== VISUAL ODOMETRY STRATEGY ==========
        'Odom/Strategy': '0',  # 0=Frame-to-Map (best for VIO with local map)
        'Odom/KeyFrameThr': '0.2',  # Create keyframe when inliers drop <20%
        'Odom/VisKeyFrameThr': '100',  # Minimum 100 inliers for keyframe
        'Odom/ImageDecimation': '1',  # No decimation (D455 can handle 640x360)

        # ========== BUNDLE ADJUSTMENT (CRITICAL FOR VIO) ==========
        'OdomF2M/BundleAdjustment': '1',  # 1=g2o bundle adjustment
        'OdomF2M/BundleAdjustmentMaxFrames': '15',  # Use last 15 frames (increased from 10)
        'OdomF2M/MaxSize': '3000',  # Larger local map = 3000 features
        'OdomF2M/ValidDepthRatio': '0.65',  # Accept frames with 65%+ valid depth
        'OdomF2M/MaxNewFeatures': '300',  # Add up to 300 new features per keyframe

        # ========== FEATURE EXTRACTION (D455 OPTIMIZED) ==========
        'Kp/DetectorStrategy': '8',  # 8=GFTT/ORB (good balance)
        'Kp/MaxFeatures': '800',  # More features = more stable (increased from 500)
        'Kp/GridRows': '3',  # 3x4 grid for uniform distribution
        'Kp/GridCols': '4',  # Matches 640x360 aspect ratio
        'Kp/MinDepth': '0.3',  # D455 minimum reliable depth (30cm)
        'Kp/MaxDepth': '6.0',  # D455 best accuracy range (<6m)
        'Kp/NndrRatio': '0.8',  # NNDR ratio for feature matching

        # ========== VISUAL FEATURE MATCHING ==========
        'Vis/FeatureType': '8',  # Should match Kp/DetectorStrategy
        'Vis/MaxFeatures': '1000',  # Extract up to 1000 features per frame
        'Vis/MinInliers': '15',  # FIXED: Lower for rotation stability (was 25)
        'Vis/InlierDistance': '0.05',  # STRICTER: Max reprojection error 5cm
        'Vis/MinDepth': '0.3',
        'Vis/MaxDepth': '6.0',

        # Correspondence matching
        'Vis/CorType': '0',  # 0=Feature Matching (more accurate than optical flow)
        'Vis/CorNNDR': '0.8',  # NNDR for matching
        'Vis/CorGuessWinSize': '40',  # Matching window around projected points

        # Estimation and refinement
        'Vis/EstimationType': '1',  # 1=3D->2D PnP (best for RGBD with IMU)
        'Vis/Iterations': '500',  # Max RANSAC iterations (increased from 300)
        'Vis/BundleAdjustment': '1',  # Enable bundle adjustment

        # Grid-based feature distribution (prevents rotation-induced loss)
        'Vis/GridRows': '3',
        'Vis/GridCols': '4',

        # Depth masking
        'Vis/DepthAsMask': 'true',
        'Mem/DepthAsMask': 'true',

        # ========== GPU ACCELERATION (NVIDIA RTX 5050 CUDA 12) ==========
        # Feature detection GPU (CUDA accelerated)
        'GFTT/Gpu': 'true',  # GPU-GFTT
        'FAST/Gpu': 'true',  # GPU-FAST
        'ORB/Gpu': 'true',  # GPU-ORB (CUDA)

        # Feature matching GPU (BruteForce-GPU)
        'Kp/NNStrategy': '4',  # 4=kNNBruteForceGPU (CUDA)
        'Vis/CorNNType': '4',  # 4=kNNBruteForceGPU (CUDA)
        'Vis/CorFlowGpu': 'true',  # GPU optical flow (CUDA)

        # ========== MEMORY AND PERFORMANCE ==========
        'Mem/ImagePreDecimation': '1',  # No decimation before feature detection
        'Mem/ImagePostDecimation': '2',  # 2x decimation for database storage
        'Mem/UseOdomFeatures': 'true',  # Reuse features across frames
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

        # Optimizer settings (same gravity constraint as odometry)
        'Optimizer/GravitySigma': '0.2',
        'Mem/UseOdomGravity': 'true',

        # Loop closure settings
        'RGBD/OptimizeMaxError': '3.0',  # Max graph error before rejecting loop closure
        'RGBD/LinearUpdate': '0.1',  # Create node every 10cm movement
        'RGBD/AngularUpdate': '0.1',  # Create node every 5.7° rotation
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data'),  # Use Madgwick filtered IMU, RTAB-Map uses TF for frame transform
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
        declare_enable_gpu,

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
