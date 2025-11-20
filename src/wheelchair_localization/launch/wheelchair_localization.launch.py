#!/usr/bin/env python3

"""
WHEELCHAIR GLOBAL LOCALIZATION (AMCL)
======================================
Launches AMCL localization using a pre-built map for the wheelchair robot.
Based on bumperbot_localization reference implementation.

Usage:
    ros2 launch wheelchair_localization wheelchair_localization.launch.py map_name:=/home/sidd/maps/wheelchair_nav.yaml

Example with custom map:
    ros2 launch wheelchair_localization wheelchair_localization.launch.py map_name:=/home/sidd/maps/my_map.yaml
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Launch configurations
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    # Lifecycle nodes for lifecycle manager
    lifecycle_nodes = ["map_server", "amcl"]

    # Declare launch arguments
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="/home/sidd/maps/wheelchair_nav.yaml",
        description="Full path to the map YAML file"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (false for real hardware)"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("wheelchair_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to AMCL yaml configuration file"
    )
    
    # Nav2 Map Server node
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_name},
            {"use_sim_time": use_sim_time}
        ],
    )

    # Nav2 AMCL node
    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Nav2 Lifecycle Manager
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        # Launch arguments
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        
        # Nodes
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
    ])
