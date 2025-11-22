#!/usr/bin/env python3
"""Social groups visualization node."""

import rclpy
from rclpy.node import Node


class SocialVizNode(Node):
    """Visualize detected social groups."""

    def __init__(self):
        super().__init__('social_viz_node')
        self.get_logger().info('Social visualization node initialized')


def main(args=None):
    rclpy.init(args=args)
    node = SocialVizNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
