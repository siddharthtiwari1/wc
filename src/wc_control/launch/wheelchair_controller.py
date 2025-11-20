from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",  # REAL HARDWARE
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    
    # Diff drive controller spawner (wc_control)
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wc_control",  # Your diff_drive_controller
            "--controller-manager", 
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Twist to TwistStamped converter for diff_drive
    twist_to_stamped = TimerAction(
        period=2.0,  # Wait for controllers to start
        actions=[
            Node(
                package="scripts",
                executable="twist_stamped_teleop",  # Your existing script
                name="twist_to_twist_stamped",
                output="screen",
                remappings=[
                    ("cmd_vel_in", "/cmd_vel"),
                    ("cmd_vel_out", "/wc_control/cmd_vel"),
                ]
            )
        ]
    )
    
    # Teleop keyboard
    teleop_keyboard = TimerAction(
        period=3.0,  # Wait a bit more
        actions=[
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                prefix="xterm -e",
                remappings=[
                    ("cmd_vel", "/cmd_vel"),
                ]
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_to_stamped,
        teleop_keyboard,
    ])
