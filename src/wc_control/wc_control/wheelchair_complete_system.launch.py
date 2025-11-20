#!/usr/bin/env python3

"""
Unified launch file for complete wheelchair system.
Launches: robot state publisher, ros2_control hardware interface, controllers,
all sensors (RealSense, IMU, RPLidar), EKF localization, and plotting.
"""

import os
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')
    wheelchair_localization_dir = get_package_share_directory('wheelchair_localization')
    wc_control_dir = get_package_share_directory('wc_control')

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
    controller_config = os.path.join(
        wc_control_dir,
        'config',
        'wc_control.yaml',
    )
    ekf_config = os.path.join(
        wheelchair_localization_dir,
        'config',
        'ekf.yaml',
    )

    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    default_is_ignition = 'true' if ros_distro == 'humble' else 'false'

    # Launch Arguments
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
    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='True when running in simulation.',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.',
    )
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for wheelchair hardware interface.',
    )
    declare_unite_imu = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='RealSense IMU synchronization (0=None, 1=copy, 2=linear interpolation).',
    )
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar.',
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
    declare_enable_plotting = DeclareLaunchArgument(
        'enable_plotting',
        default_value='true',
        description='Enable square path EKF plotting.',
    )

    # Robot Description with control parameters
    robot_description = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('model'),
            ' is_ignition:=',
            LaunchConfiguration('is_ignition'),
            ' is_sim:=',
            LaunchConfiguration('is_sim'),
            ' port:=',
            LaunchConfiguration('port'),
        ]),
        value_type=str,
    )

    # Robot State Publisher
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

    # Joint State Publisher (optional, for GUI control of non-actuated joints)
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
            joint_state_actions.append(LogInfo(msg='No joint state publisher package found; continuing without.'))

    # USB Permissions Setup
    def _grant_usb_permissions(context, *_args, **_kwargs):
        """Ensure serial devices are writable before nodes start."""
        lidar_port = LaunchConfiguration('lidar_port').perform(context)
        wheelchair_port = LaunchConfiguration('port').perform(context)
        password = LaunchConfiguration('sudo_password').perform(context)

        for port in [lidar_port, wheelchair_port]:
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

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    # Spawn Joint State Broadcaster
    spawn_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )

    # Spawn Diff Drive Controller
    spawn_wc_control = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['wc_control'],
                output='screen',
            )
        ]
    )

    # RealSense Camera Launch
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

    # IMU Filter (Madgwick)
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

    # IMU Republisher (transforms to base_link frame)
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

    # RPLidar S3
    rplidar_s3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s3_launch.py',
            )
        ]),
    )

    # EKF Localization Node
    ekf_localization_node = TimerAction(
        period=5.0,  # Wait for sensors to start
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config,
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ]
            )
        ]
    )

    # Square Path EKF Tester/Plotter
    square_path_plotter = Node(
        package='scripts',
        executable='square_path_ekf_tester',
        name='square_path_ekf_tester',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_plotting')),
    )

    # RViz
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
        declare_port,
        declare_unite_imu,
        declare_lidar_port,
        declare_sudo_password,
        declare_rviz,
        declare_rviz_config,
        declare_enable_plotting,

        # USB permissions
        permission_setup,

        # Robot state publisher
        robot_state_publisher,
        *joint_state_actions,
        *([joint_state_publisher] if joint_state_publisher else []),

        # ros2_control and controllers
        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_wc_control,

        # Sensors
        realsense_launch,
        imu_filter,
        imu_wheelchair_republisher,
        rplidar_s3_launch,

        # Localization
        ekf_localization_node,

        # Visualization and plotting
        square_path_plotter,
        rviz_node,
    ])
