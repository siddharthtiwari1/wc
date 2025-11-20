#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Detect ROS distro for Gazebo compatibility
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    is_ignition = "true" if ros_distro == "humble" else "false"

    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Whether to run in simulation mode (true) or real hardware (false)'
    )

    port = LaunchConfiguration('port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_sim = LaunchConfiguration('is_sim')

    # Robot description - configurable for sim or real hardware
    robot_description_content = Command([
        'xacro ',
        os.path.join(get_package_share_directory('wheelchair_description'), 'urdf', 'wheelchair_description.urdf.xacro'),
        ' is_sim:=', is_sim,
        ' is_ignition:=', is_ignition,
        ' port:=', port
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Hardware Interface + Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory('wc_control'), 'config', 'wc_control.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Joint State Broadcaster - Wait for hardware to initialize
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,  # Wait for hardware interface
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # wc_control Differential Drive Controller - Wait for joint state broadcaster
    wc_control_spawner = TimerAction(
        period=7.0,  # Wait for joint state broadcaster
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['wc_control'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # Optional: Teleop for testing
    teleop_keyboard = TimerAction(
        period=10.0,  # Wait for all controllers
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[
                    ('cmd_vel', '/wc_control/cmd_vel'),  # Direct to wc_control
                ]
            )
        ]
    )

    return LaunchDescription([
        port_arg,
        use_sim_time_arg,
        is_sim_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        wc_control_spawner,
        # teleop_keyboard,  # Removed - you'll use transmitter for real robot
    ])