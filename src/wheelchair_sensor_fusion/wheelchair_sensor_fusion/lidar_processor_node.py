#!/usr/bin/env python3
"""LiDAR Processor Node for Wheelchair Sensor Fusion.

This node processes 2D LiDAR scan data from RPLidar S3, performs clustering
using DBSCAN, and publishes obstacle clusters for sensor fusion.

Author: Siddharth Tiwari
Email: s24035@students.iitmandi.ac.in
"""

import numpy as np
from sklearn.cluster import DBSCAN
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math


class LidarProcessorNode(Node):
    """Process LiDAR scans and extract obstacle clusters."""

    def __init__(self):
        """Initialize the LiDAR processor node."""
        super().__init__('lidar_processor_node')

        # Declare parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('clusters_topic', '/lidar/clusters')
        self.declare_parameter('visualization_topic', '/lidar/clusters_viz')
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('dbscan_eps', 0.15)
        self.declare_parameter('dbscan_min_samples', 3)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('max_cluster_size', 100)
        self.declare_parameter('publish_rate', 20.0)

        # Get parameters
        self.scan_topic = self.get_parameter('scan_topic').value
        self.clusters_topic = self.get_parameter('clusters_topic').value
        self.viz_topic = self.get_parameter('visualization_topic').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value

        # Create publishers
        self.cluster_pub = self.create_publisher(
            MarkerArray,
            self.clusters_topic,
            10
        )
        self.viz_pub = self.create_publisher(
            MarkerArray,
            self.viz_topic,
            10
        )

        # Create subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # DBSCAN clusterer
        self.clusterer = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_samples,
            metric='euclidean'
        )

        # Statistics
        self.scan_count = 0
        self.cluster_count = 0

        self.get_logger().info(
            f'LiDAR Processor initialized - Listening to {self.scan_topic}'
        )

    def scan_callback(self, msg: LaserScan):
        """Process incoming LaserScan messages.

        Args:
            msg: LaserScan message from LiDAR sensor
        """
        self.scan_count += 1

        # Convert polar coordinates to Cartesian
        points = self.scan_to_cartesian(msg)

        if len(points) < self.min_cluster_size:
            self.get_logger().warn('Insufficient valid points for clustering')
            return

        # Perform DBSCAN clustering
        clusters = self.cluster_points(points)

        # Filter clusters by size
        valid_clusters = [
            c for c in clusters
            if self.min_cluster_size <= len(c) <= self.max_cluster_size
        ]

        if not valid_clusters:
            # Publish empty marker array
            empty_array = MarkerArray()
            self.cluster_pub.publish(empty_array)
            self.viz_pub.publish(empty_array)
            return

        # Publish clusters as markers
        cluster_markers = self.create_cluster_markers(
            valid_clusters,
            msg.header
        )
        self.cluster_pub.publish(cluster_markers)

        # Publish visualization markers
        viz_markers = self.create_visualization_markers(
            valid_clusters,
            msg.header
        )
        self.viz_pub.publish(viz_markers)

        self.cluster_count += len(valid_clusters)

        if self.scan_count % 100 == 0:
            self.get_logger().info(
                f'Processed {self.scan_count} scans, '
                f'detected {self.cluster_count} total clusters'
            )

    def scan_to_cartesian(self, scan: LaserScan) -> np.ndarray:
        """Convert LaserScan polar coordinates to Cartesian.

        Args:
            scan: LaserScan message

        Returns:
            Nx2 numpy array of (x, y) coordinates
        """
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            # Filter by range limits
            if self.min_range <= r <= self.max_range and not math.isinf(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])

            angle += scan.angle_increment

        return np.array(points) if points else np.array([]).reshape(0, 2)

    def cluster_points(self, points: np.ndarray) -> list:
        """Cluster points using DBSCAN algorithm.

        Args:
            points: Nx2 numpy array of (x, y) coordinates

        Returns:
            List of clusters, where each cluster is a list of point indices
        """
        if len(points) == 0:
            return []

        # Perform DBSCAN clustering
        labels = self.clusterer.fit_predict(points)

        # Group points by cluster label (ignore noise labeled as -1)
        clusters = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:  # Skip noise
                continue

            cluster_indices = np.where(labels == label)[0]
            cluster_points = points[cluster_indices]
            clusters.append(cluster_points)

        return clusters

    def create_cluster_markers(
        self,
        clusters: list,
        header: Header
    ) -> MarkerArray:
        """Create marker array representing cluster centroids and bounding boxes.

        Args:
            clusters: List of clusters (each is Nx2 numpy array)
            header: ROS message header

        Returns:
            MarkerArray message
        """
        marker_array = MarkerArray()

        for i, cluster in enumerate(clusters):
            # Compute cluster centroid
            centroid = np.mean(cluster, axis=0)

            # Compute bounding box
            min_pt = np.min(cluster, axis=0)
            max_pt = np.max(cluster, axis=0)
            size = max_pt - min_pt

            # Create centroid marker
            centroid_marker = Marker()
            centroid_marker.header = header
            centroid_marker.ns = 'cluster_centroids'
            centroid_marker.id = i
            centroid_marker.type = Marker.SPHERE
            centroid_marker.action = Marker.ADD
            centroid_marker.pose.position.x = float(centroid[0])
            centroid_marker.pose.position.y = float(centroid[1])
            centroid_marker.pose.position.z = 0.0
            centroid_marker.pose.orientation.w = 1.0
            centroid_marker.scale.x = 0.1
            centroid_marker.scale.y = 0.1
            centroid_marker.scale.z = 0.1
            centroid_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            centroid_marker.lifetime.sec = 0
            centroid_marker.lifetime.nanosec = 100000000  # 0.1 seconds

            marker_array.markers.append(centroid_marker)

            # Create bounding box marker
            bbox_marker = Marker()
            bbox_marker.header = header
            bbox_marker.ns = 'cluster_bboxes'
            bbox_marker.id = i
            bbox_marker.type = Marker.CUBE
            bbox_marker.action = Marker.ADD
            bbox_marker.pose.position.x = float((min_pt[0] + max_pt[0]) / 2)
            bbox_marker.pose.position.y = float((min_pt[1] + max_pt[1]) / 2)
            bbox_marker.pose.position.z = 0.0
            bbox_marker.pose.orientation.w = 1.0
            bbox_marker.scale.x = float(max(size[0], 0.05))
            bbox_marker.scale.y = float(max(size[1], 0.05))
            bbox_marker.scale.z = 0.1
            bbox_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
            bbox_marker.lifetime.sec = 0
            bbox_marker.lifetime.nanosec = 100000000

            marker_array.markers.append(bbox_marker)

        return marker_array

    def create_visualization_markers(
        self,
        clusters: list,
        header: Header
    ) -> MarkerArray:
        """Create detailed visualization markers for RViz.

        Args:
            clusters: List of clusters
            header: ROS message header

        Returns:
            MarkerArray message
        """
        marker_array = MarkerArray()

        for i, cluster in enumerate(clusters):
            # Create point cloud marker for each cluster
            points_marker = Marker()
            points_marker.header = header
            points_marker.ns = 'cluster_points'
            points_marker.id = i
            points_marker.type = Marker.POINTS
            points_marker.action = Marker.ADD
            points_marker.pose.orientation.w = 1.0
            points_marker.scale.x = 0.05
            points_marker.scale.y = 0.05

            # Color based on cluster ID
            color = self.get_cluster_color(i)
            points_marker.color = color
            points_marker.lifetime.sec = 0
            points_marker.lifetime.nanosec = 100000000

            # Add all points
            for pt in cluster:
                p = Point()
                p.x = float(pt[0])
                p.y = float(pt[1])
                p.z = 0.0
                points_marker.points.append(p)

            marker_array.markers.append(points_marker)

        return marker_array

    def get_cluster_color(self, cluster_id: int) -> ColorRGBA:
        """Generate a unique color for each cluster.

        Args:
            cluster_id: Cluster identifier

        Returns:
            ColorRGBA message
        """
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
        ]
        idx = cluster_id % len(colors)
        return ColorRGBA(
            r=colors[idx][0],
            g=colors[idx][1],
            b=colors[idx][2],
            a=0.8
        )


def main(args=None):
    """Run the LiDAR processor node."""
    rclpy.init(args=args)
    node = LidarProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down - Processed {node.scan_count} scans, '
            f'detected {node.cluster_count} clusters'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
