import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    wheelchair_description_dir = get_package_share_directory("wheelchair_description")

    # Detect ROS distro for compatibility
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    is_ignition = "true" if ros_distro == "humble" else "false"

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        wheelchair_description_dir, "urdf", "wheelchair_description.urdf"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_actions = []
    # Prefer GUI joint state publisher; fall back to CLI version if GUI missing.
    joint_state_publisher_node = None

    try:
        get_package_share_directory("joint_state_publisher_gui")
        joint_state_publisher_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        )
    except PackageNotFoundError:
        try:
            get_package_share_directory("joint_state_publisher")
            joint_state_actions.append(LogInfo(msg="joint_state_publisher_gui not found; using joint_state_publisher."))
            joint_state_publisher_node = Node(
                package="joint_state_publisher",
                executable="joint_state_publisher"
            )
        except PackageNotFoundError:
            joint_state_actions.append(LogInfo(msg="No joint state publisher package found; skipping joint state publisher node."))

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(wheelchair_description_dir, "rviz", "urdf_config.rviz")],
    )

    launch_actions = [model_arg, *joint_state_actions]

    if joint_state_publisher_node:
        launch_actions.append(joint_state_publisher_node)

    launch_actions.extend([robot_state_publisher_node, rviz_node])

    return LaunchDescription(launch_actions)
