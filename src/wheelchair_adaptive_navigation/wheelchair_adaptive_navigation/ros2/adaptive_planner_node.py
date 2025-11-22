#!/usr/bin/env python3
"""
ROS2 node for adaptive diffusion-based crowd navigation.
"""

import rclpy
from rclpy.node import Node
import torch

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan

from wheelchair_adaptive_navigation.models.diffusion import DiffusionModel


class AdaptivePlannerNode(Node):
    """Main ROS2 node for adaptive navigation."""

    def __init__(self):
        super().__init__('adaptive_planner_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('ensemble_size', 10)

        device = self.get_parameter('device').value

        self.get_logger().info(f'Adaptive Planner initialized on {device}')

        # Initialize diffusion model
        self.diffusion = DiffusionModel(
            trajectory_dim=2,
            horizon=30,
            condition_dim=512,
        ).to(device)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/local_plan', 10)

        self.get_logger().info('Adaptive navigation node ready')

    def scan_callback(self, msg):
        """Handle laser scan."""
        pass  # TODO: Implement planning


def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
