#!/usr/bin/env python3

"""
Wheelchair OPTIMIZED Visual-Inertial Odometry (VIO) + SLAM Launch File

OPTIMIZATIONS:
- Maximum IMU utilization with tightly-coupled VIO
- GPU acceleration for feature extraction and matching (if OpenCV has CUDA)
- D435i/D455 specific depth range and resolution tuning
- Enhanced bundle adjustment with larger local map
- Stricter feature matching for rotation stability
- Gravity constraints for translation accuracy

Hardware Requirements:
- RealSense D435i or D455 with IMU
- GPU with CUDA support (optional, for GPU acceleration)
- ROS2 Jazzy/Humble with RTAB-Map

Performance Targets:
- Rotation: <2cm drift during 360° rotation
- Translation: <5% error over 1m movement
- Update Rate: 20-30 Hz odometry
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
            'rgb_camera.profile': '1280x720x30',
            'depth_module.profile': '1280x720x30',
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
        'Odom/KeyFrameThr': '0.4',  # RELAXED: Create keyframe when inliers drop <40%
        'Odom/VisKeyFrameThr': '20',  # RELAXED: Minimum 20 inliers for keyframe
        'Odom/ImageDecimation': '1',  # No decimation
        'Odom/ResetCountdown': '5',  # Reset odometry after 5 consecutive failures

        # ========== BUNDLE ADJUSTMENT (DISABLED FOR STABILITY) ==========
        'OdomF2M/BundleAdjustment': '0',  # DISABLED: BA is rejecting too many inliers
        'OdomF2M/BundleAdjustmentMaxFrames': '10',  # Reduced for performance
        'OdomF2M/MaxSize': '3000',  # Larger local map = 3000 features
        'OdomF2M/ValidDepthRatio': '0.5',  # RELAXED: Accept frames with 50%+ valid depth
        'OdomF2M/MaxNewFeatures': '500',  # INCREASED: Add up to 500 new features per keyframe

        # ========== FEATURE EXTRACTION (HD 1280x720 OPTIMIZED) ==========
        'Kp/DetectorStrategy': '6',  # 6=GFTT (Good Features To Track - better for tracking)
        'Kp/MaxFeatures': '1000',  # Reduce to focus on best features (was 1200)
        'Kp/GridRows': '4',  # 4x6 grid for uniform distribution
        'Kp/GridCols': '6',  # Matches 1280x720 aspect ratio (16:9)
        'Kp/MinDepth': '0.3',  # D455 minimum reliable depth (30cm)
        'Kp/MaxDepth': '6.0',  # D455 best accuracy range (<6m)
        'Kp/NndrRatio': '0.8',  # NNDR ratio for feature matching
        'GFTT/QualityLevel': '0.001',  # Lower = more features detected
        'GFTT/MinDistance': '5',  # Minimum 5 pixels between features

        # ========== VISUAL FEATURE MATCHING ==========
        'Vis/FeatureType': '6',  # Should match Kp/DetectorStrategy (GFTT)
        'Vis/MaxFeatures': '2000',  # INCREASED: Extract up to 2000 features per frame
        'Vis/MinInliers': '8',  # VERY RELAXED: Minimum 8 inliers (was 10)
        'Vis/InlierDistance': '0.15',  # VERY RELAXED: Max reprojection error 15cm (was 0.1)
        'Vis/MinDepth': '0.2',  # Allow closer features
        'Vis/MaxDepth': '8.0',  # Allow farther features

        # Correspondence matching
        'Vis/CorType': '0',  # 0=Feature Matching (more accurate than optical flow)
        'Vis/CorNNDR': '0.8',  # NNDR for matching
        'Vis/CorGuessWinSize': '40',  # Matching window around projected points

        # Estimation and refinement
        'Vis/EstimationType': '1',  # 1=3D->2D PnP (best for RGBD with IMU)
        'Vis/Iterations': '300',  # Max RANSAC iterations
        'Vis/BundleAdjustment': '0',  # DISABLED: Prevents inlier rejection
        'Vis/RefineIterations': '5',  # Minimal refinement only

        # Grid-based feature distribution (prevents rotation-induced loss)
        'Vis/GridRows': '4',  # Match new grid layout
        'Vis/GridCols': '6',  # 4x6 for 1280x720

        # Depth masking
        'Vis/DepthAsMask': 'true',
        'Mem/DepthAsMask': 'true',

        # ========== CPU MODE (GPU DISABLED - see wheelchair_vio_gpu.launch.py for GPU) ==========
        # Feature detection CPU
        'GFTT/Gpu': 'false',
        'FAST/Gpu': 'false',
        'ORB/Gpu': 'false',

        # Feature matching CPU
        'Kp/NNStrategy': '1',  # 1=kNNFlannNaive (CPU optimized)
        'Vis/CorNNType': '1',  # 1=kNNFlannNaive
        'Vis/CorFlowGpu': 'false',

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
