#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM1',
        description='Arduino port')

    port = LaunchConfiguration('port')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        os.path.join(get_package_share_directory('wheelchair_description'), 'urdf', 'wheelchair_description.urdf.xacro'),
        ' is_sim:=false',
        ' port:=', port
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        output='screen'
    )

    # Controller Manager + Hardware Interface
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory('wc_control'), 'config', 'wc_control.yaml'),
            {'use_sim_time': False}
        ],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen'
    )

    # WC Control Controller Spawner
    wc_control_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wc_control', '--controller-manager-timeout', '30'],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        wc_control_spawner,
    ])