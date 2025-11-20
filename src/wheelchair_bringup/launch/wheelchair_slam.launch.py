#!/usr/bin/env python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    rviz_config = LaunchConfiguration("rviz_config")

    wheelchair_localization_dir = get_package_share_directory('wheelchair_localization')
    wheelchair_description_dir = get_package_share_directory('wheelchair_description')

    # SLAM config from wheelchair_localization
    default_slam_config = os.path.join(
        wheelchair_localization_dir,
        'config',
        'slam_toolbox.yaml'
    )

    # RViz SLAM mapping config
    default_rviz_config = os.path.join(
        wheelchair_description_dir,
        'rviz',
        'slam_mapping.rviz'
    )

    # ROS distro detection (slam_toolbox lifecycle differs in Humble vs Jazzy)
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    lifecycle_nodes = ["map_saver_server"]
    if ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=default_slam_config,
        description="Full path to slam_toolbox yaml config"
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Full path to RViz config file"
    )

    # ========================================================================
    # MAP SAVER SERVER
    # ========================================================================

    map_saver_server = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
        ],
    )

    # ========================================================================
    # SLAM TOOLBOX
    # ========================================================================

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ========================================================================
    # LIFECYCLE MANAGER
    # ========================================================================

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    # ========================================================================
    # RVIZ2
    # ========================================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        rviz_config_arg,
        map_saver_server,
        slam_toolbox,
        lifecycle_manager,
        rviz_node,
    ])
