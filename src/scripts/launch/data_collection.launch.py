from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Data collector node
        Node(
            package='scripts',
            executable='data_collector.py',
            name='wheelchair_data_collector',
            output='screen',
            parameters=[
                {'log_frequency': 10.0}  # 10 Hz logging
            ]
        ),
        
        # Optional: Launch rviz for visualization
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(os.getcwd(), 'wheelchair_data_viz.rviz')]
        # )
    ])