#!/usr/bin/env python3
"""Obstacle Publisher Node for Nav2 Integration.

This node converts fused obstacle markers to costmap obstacles
for Nav2 obstacle avoidance.

Author: Siddharth Tiwari
Email: s24035@students.iitmandi.ac.in
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np


class ObstaclePublisherNode(Node):
    """Publish obstacles to Nav2 costmap."""

    def __init__(self):
        """Initialize the obstacle publisher node."""
        super().__init__('obstacle_publisher_node')

        # Declare parameters
        self.declare_parameter('obstacles_topic', '/fusion/obstacles')
        self.declare_parameter('costmap_topic', '/fusion/costmap')
        self.declare_parameter('grid_resolution', 0.05)  # 5cm cells
        self.declare_parameter('grid_width', 10.0)  # meters
        self.declare_parameter('grid_height', 10.0)  # meters
        self.declare_parameter('obstacle_inflation', 0.3)  # meters

        # Get parameters
        self.obstacles_topic = self.get_parameter('obstacles_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.resolution = self.get_parameter('grid_resolution').value
        self.width = self.get_parameter('grid_width').value
        self.height = self.get_parameter('grid_height').value
        self.inflation = self.get_parameter('obstacle_inflation').value

        # Grid dimensions
        self.grid_width = int(self.width / self.resolution)
        self.grid_height = int(self.height / self.resolution)

        # Publishers
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.costmap_topic,
            10
        )

        # Subscribers
        self.obstacles_sub = self.create_subscription(
            MarkerArray,
            self.obstacles_topic,
            self.obstacles_callback,
            10
        )

        self.get_logger().info(
            f'Obstacle Publisher initialized - Publishing to {self.costmap_topic}'
        )

    def obstacles_callback(self, msg: MarkerArray):
        """Convert obstacles to costmap.

        Args:
            msg: MarkerArray of obstacles
        """
        # Create occupancy grid
        costmap = OccupancyGrid()
        costmap.header = msg.markers[0].header if msg.markers else None
        costmap.info.resolution = self.resolution
        costmap.info.width = self.grid_width
        costmap.info.height = self.grid_height
        costmap.info.origin.position.x = -self.width / 2
        costmap.info.origin.position.y = -self.height / 2
        costmap.info.origin.orientation.w = 1.0

        # Initialize grid (0 = free, 100 = occupied)
        grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        # Mark obstacles
        for marker in msg.markers:
            if marker.ns == 'fused_obstacles':
                self.mark_obstacle(
                    grid,
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.scale.x,
                    marker.scale.y
                )

        # Flatten and publish
        costmap.data = grid.flatten().tolist()
        self.costmap_pub.publish(costmap)

    def mark_obstacle(
        self,
        grid: np.ndarray,
        x: float,
        y: float,
        size_x: float,
        size_y: float
    ):
        """Mark obstacle in grid.

        Args:
            grid: Occupancy grid
            x, y: Obstacle position
            size_x, size_y: Obstacle size
        """
        # Convert to grid coordinates
        gx = int((x + self.width / 2) / self.resolution)
        gy = int((y + self.height / 2) / self.resolution)

        # Obstacle size in grid cells
        half_x = int((size_x / 2 + self.inflation) / self.resolution)
        half_y = int((size_y / 2 + self.inflation) / self.resolution)

        # Mark cells
        for dx in range(-half_x, half_x + 1):
            for dy in range(-half_y, half_y + 1):
                nx = gx + dx
                ny = gy + dy

                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                    grid[ny, nx] = 100  # Occupied


def main(args=None):
    """Run the obstacle publisher node."""
    rclpy.init(args=args)
    node = ObstaclePublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
