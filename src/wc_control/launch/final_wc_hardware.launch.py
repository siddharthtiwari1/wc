#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    is_sim_arg = DeclareLaunchArgument(
        'is_sim', default_value='true',
        description='Use simulation')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', 
        description='Use simulation time')
        
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM0',
        description='Arduino port')

    is_sim = LaunchConfiguration('is_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        os.path.join(get_package_share_directory('wheelchair_description'), 'urdf', 'wheelchair_description.urdf.xacro'),
        ' is_sim:=', is_sim,
        ' port:=', port
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Global use_sim_time parameter override
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Controller Manager + Hardware Interface
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

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    # WC Control Controller 
    wc_control_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['wc_control'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        use_sim_time_arg,
        port_arg,
        set_use_sim_time,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        wc_control_spawner,
    ])