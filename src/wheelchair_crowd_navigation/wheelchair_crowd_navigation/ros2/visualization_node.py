#!/usr/bin/env python3
"""
Visualization node for crowd navigation.

Publishes MarkerArrays for RViz2 visualization.
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class VisualizationNode(Node):
    """Node for visualizing crowd navigation."""

    def __init__(self):
        super().__init__('visualization_node')

        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/local_plan',
            self.path_callback,
            10
        )

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/crowd_viz',
            10
        )

        self.get_logger().info('Visualization node initialized')

    def path_callback(self, msg: Path):
        """Visualize planned path."""
        marker_array = MarkerArray()

        # Path line marker
        path_marker = Marker()
        path_marker.header = msg.header
        path_marker.ns = 'planned_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05

        path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        for pose in msg.poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = pose.pose.position.z
            path_marker.points.append(point)

        marker_array.markers.append(path_marker)

        # Publish
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = VisualizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
