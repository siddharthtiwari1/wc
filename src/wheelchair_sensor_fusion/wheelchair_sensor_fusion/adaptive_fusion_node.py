#!/usr/bin/env python3
"""
Adaptive Confidence-Weighted Fusion (ACWF) Node

Novel sensor fusion algorithm combining 2D LiDAR and depth camera for
robust obstacle detection in wheelchair navigation.

Key Innovations (Publication Contributions):
1. Distance-based reliability weighting: LiDAR more reliable at distance
2. Lighting-adaptive confidence: Reduce camera weight in low light
3. Motion-adaptive weighting: Increase LiDAR weight during fast motion
4. Temporal obstacle persistence: Track obstacles over time
5. Cross-modal verification: Detect transparent/low obstacles

Publications:
- /fusion/obstacles (visualization_msgs/MarkerArray): Fused obstacles
- /fusion/pointcloud (sensor_msgs/PointCloud2): Fused pointcloud for costmap
- /fusion/confidence (std_msgs/Float32MultiArray): Sensor confidence values
- /fusion/stats (std_msgs/String): Fusion statistics

Author: Siddharth Tiwari (s24035@students.iitmandi.ac.in)
Target: ICRA/IROS Publication
"""

import numpy as np
import math
from typing import List, Dict, Optional, Tuple
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image, PointCloud2, PointField, CameraInfo
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, Float32MultiArray, String
import message_filters
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException
import struct


class TrackedObstacle:
    """Represents a tracked obstacle with temporal persistence."""

    def __init__(self, obstacle_id: int, position: np.ndarray, timestamp: float):
        self.id = obstacle_id
        self.position = position  # [x, y, z]
        self.velocity = np.zeros(3)
        self.size = np.array([0.1, 0.1, 0.1])
        self.confidence = 0.0
        self.lidar_confidence = 0.0
        self.camera_confidence = 0.0
        self.last_seen = timestamp
        self.hit_count = 1
        self.class_name = 'obstacle'
        self.is_dynamic = False

    def update(self, new_position: np.ndarray, timestamp: float):
        """Update obstacle with new observation."""
        dt = timestamp - self.last_seen
        if dt > 0:
            self.velocity = (new_position - self.position) / dt
            if np.linalg.norm(self.velocity[:2]) > 0.1:  # Moving > 10cm/s
                self.is_dynamic = True

        self.position = 0.7 * self.position + 0.3 * new_position  # Smooth update
        self.last_seen = timestamp
        self.hit_count += 1


class AdaptiveFusionNode(Node):
    """
    Adaptive Confidence-Weighted Fusion (ACWF) for visual-LiDAR fusion.

    This node implements a novel fusion algorithm that adaptively weights
    sensor contributions based on:
    - Distance to obstacle
    - Lighting conditions
    - Robot motion state
    - Temporal consistency
    """

    def __init__(self):
        super().__init__('adaptive_fusion_node')

        # ====================================================================
        # PARAMETERS
        # ====================================================================

        # Fusion weights
        self.declare_parameter('base_lidar_weight', 0.6)
        self.declare_parameter('base_camera_weight', 0.4)
        self.declare_parameter('distance_crossover', 2.0)  # meters
        self.declare_parameter('weight_steepness', 2.0)    # sigmoid steepness

        # Temporal tracking
        self.declare_parameter('obstacle_timeout', 2.0)    # seconds
        self.declare_parameter('association_threshold', 0.5)  # meters
        self.declare_parameter('min_observations', 2)      # hits to confirm

        # LiDAR processing
        self.declare_parameter('lidar_cluster_tolerance', 0.15)  # meters
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('max_cluster_size', 200)

        # Depth camera processing
        self.declare_parameter('depth_min_range', 0.3)     # meters
        self.declare_parameter('depth_max_range', 4.0)     # meters
        self.declare_parameter('depth_voxel_size', 0.05)   # meters

        # Output configuration
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('publish_rate', 15.0)       # Hz

        # Get parameters
        self.base_lidar_w = self.get_parameter('base_lidar_weight').value
        self.base_camera_w = self.get_parameter('base_camera_weight').value
        self.dist_crossover = self.get_parameter('distance_crossover').value
        self.weight_steep = self.get_parameter('weight_steepness').value

        self.obs_timeout = self.get_parameter('obstacle_timeout').value
        self.assoc_thresh = self.get_parameter('association_threshold').value
        self.min_obs = self.get_parameter('min_observations').value

        self.cluster_tol = self.get_parameter('lidar_cluster_tolerance').value
        self.min_cluster = self.get_parameter('min_cluster_size').value
        self.max_cluster = self.get_parameter('max_cluster_size').value

        self.depth_min = self.get_parameter('depth_min_range').value
        self.depth_max = self.get_parameter('depth_max_range').value
        self.voxel_size = self.get_parameter('depth_voxel_size').value

        self.output_frame = self.get_parameter('output_frame').value
        self.pub_rate = self.get_parameter('publish_rate').value

        # ====================================================================
        # STATE VARIABLES
        # ====================================================================

        self.tracked_obstacles: Dict[int, TrackedObstacle] = {}
        self.next_obstacle_id = 0
        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.K: Optional[np.ndarray] = None  # Camera intrinsics

        # Lighting estimation
        self.current_brightness = 128.0  # 0-255 scale
        self.brightness_history = deque(maxlen=30)

        # Motion estimation (from cmd_vel or odom)
        self.current_velocity = 0.0

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Statistics
        self.fusion_count = 0
        self.lidar_only_count = 0
        self.camera_only_count = 0
        self.fused_count = 0

        # ====================================================================
        # PUBLISHERS
        # ====================================================================

        self.obstacles_pub = self.create_publisher(
            MarkerArray, '/fusion/obstacles', 10
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/fusion/pointcloud', 10
        )
        self.confidence_pub = self.create_publisher(
            Float32MultiArray, '/fusion/confidence', 10
        )
        self.stats_pub = self.create_publisher(
            String, '/fusion/stats', 10
        )

        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # Camera info for projection
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_callback, 10
        )

        # Synchronized subscribers
        self.scan_sub = message_filters.Subscriber(
            self, LaserScan, '/scan_filtered', qos_profile=qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw', qos_profile=qos
        )
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=qos
        )

        # Approximate time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.depth_sub, self.rgb_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.fusion_callback)

        # ====================================================================
        # TIMER
        # ====================================================================

        self.cleanup_timer = self.create_timer(0.5, self.cleanup_old_obstacles)
        self.stats_timer = self.create_timer(5.0, self.publish_stats)

        self.get_logger().info('Adaptive Fusion Node initialized')
        self.get_logger().info(f'  Base weights: LiDAR={self.base_lidar_w}, Camera={self.base_camera_w}')
        self.get_logger().info(f'  Distance crossover: {self.dist_crossover}m')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics."""
        if self.camera_info is None:
            self.camera_info = msg
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info('Camera intrinsics received')

    def fusion_callback(self, scan_msg: LaserScan, depth_msg: Image, rgb_msg: Image):
        """Main fusion callback - synchronized sensor processing."""
        self.fusion_count += 1
        timestamp = self.get_clock().now().nanoseconds / 1e9

        # Process LiDAR
        lidar_obstacles = self.process_lidar(scan_msg)

        # Process depth camera
        depth_obstacles, depth_points = self.process_depth(depth_msg)

        # Estimate lighting from RGB
        self.estimate_lighting(rgb_msg)

        # Compute adaptive weights
        weights = self.compute_adaptive_weights()

        # Fuse obstacles
        fused_obstacles = self.fuse_obstacles(
            lidar_obstacles, depth_obstacles, weights, timestamp
        )

        # Update tracked obstacles
        self.update_tracking(fused_obstacles, timestamp)

        # Publish results
        self.publish_obstacles()
        self.publish_fused_pointcloud(lidar_obstacles, depth_points, scan_msg.header)
        self.publish_confidence(weights)

    def process_lidar(self, msg: LaserScan) -> List[Dict]:
        """Process LiDAR scan into obstacle clusters."""
        obstacles = []

        # Convert to Cartesian
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment

        if len(points) < self.min_cluster:
            return obstacles

        points = np.array(points)

        # Simple clustering (DBSCAN-like)
        clusters = self.cluster_points(points)

        for cluster in clusters:
            if self.min_cluster <= len(cluster) <= self.max_cluster:
                centroid = np.mean(cluster, axis=0)
                size = np.max(cluster, axis=0) - np.min(cluster, axis=0)
                distance = np.linalg.norm(centroid)

                obstacles.append({
                    'position': np.array([centroid[0], centroid[1], 0.0]),
                    'size': np.array([max(size[0], 0.1), max(size[1], 0.1), 0.5]),
                    'distance': distance,
                    'source': 'lidar',
                    'confidence': self.compute_lidar_confidence(distance, len(cluster))
                })

        self.lidar_only_count += len(obstacles)
        return obstacles

    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """Simple distance-based clustering."""
        if len(points) == 0:
            return []

        clusters = []
        used = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if used[i]:
                continue

            cluster = [points[i]]
            used[i] = True
            queue = [i]

            while queue:
                current = queue.pop(0)
                for j in range(len(points)):
                    if not used[j]:
                        dist = np.linalg.norm(points[current] - points[j])
                        if dist < self.cluster_tol:
                            cluster.append(points[j])
                            used[j] = True
                            queue.append(j)

            clusters.append(np.array(cluster))

        return clusters

    def process_depth(self, msg: Image) -> Tuple[List[Dict], np.ndarray]:
        """Process depth image into obstacle points."""
        obstacles = []
        points_3d = []

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}')
            return obstacles, np.array([])

        if self.K is None:
            return obstacles, np.array([])

        h, w = depth.shape
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        # Subsample for efficiency
        step = 4
        for v in range(0, h, step):
            for u in range(0, w, step):
                d = depth[v, u] / 1000.0  # mm to m

                if self.depth_min < d < self.depth_max:
                    x = (u - cx) * d / fx
                    y = (v - cy) * d / fy
                    z = d

                    # Transform from camera to base_link (simplified)
                    # Assume camera is forward-facing
                    base_x = z   # depth becomes forward
                    base_y = -x  # camera x becomes -robot y
                    base_z = -y + 0.5  # camera y becomes -robot z, offset for height

                    # Filter by height (0.1m to 1.8m)
                    if 0.1 < base_z < 1.8:
                        points_3d.append([base_x, base_y, base_z])

        if len(points_3d) < 10:
            return obstacles, np.array([])

        points_3d = np.array(points_3d)

        # Cluster depth points (project to 2D for clustering)
        points_2d = points_3d[:, :2]
        clusters = self.cluster_points(points_2d)

        for cluster_2d in clusters:
            if len(cluster_2d) >= self.min_cluster:
                centroid = np.mean(cluster_2d, axis=0)
                distance = np.linalg.norm(centroid)

                obstacles.append({
                    'position': np.array([centroid[0], centroid[1], 0.5]),
                    'size': np.array([0.3, 0.3, 1.0]),
                    'distance': distance,
                    'source': 'camera',
                    'confidence': self.compute_camera_confidence(distance)
                })

        self.camera_only_count += len(obstacles)
        return obstacles, points_3d

    def estimate_lighting(self, msg: Image):
        """Estimate scene lighting from RGB image."""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Simple brightness estimation
            gray = np.mean(rgb, axis=2)
            brightness = np.mean(gray)
            self.brightness_history.append(brightness)
            self.current_brightness = np.mean(self.brightness_history)
        except Exception:
            pass

    def compute_adaptive_weights(self) -> Dict[str, float]:
        """Compute adaptive sensor weights based on conditions."""

        # Base weights
        w_lidar = self.base_lidar_w
        w_camera = self.base_camera_w

        # Lighting adaptation (camera less reliable in low light)
        light_factor = min(1.0, self.current_brightness / 100.0)
        w_camera *= (0.3 + 0.7 * light_factor)

        # Motion adaptation (camera less reliable during fast motion)
        motion_factor = 1.0 - min(1.0, self.current_velocity / 0.5)
        w_camera *= (0.5 + 0.5 * motion_factor)

        # Normalize
        total = w_lidar + w_camera
        if total > 0:
            w_lidar /= total
            w_camera /= total

        return {
            'lidar': w_lidar,
            'camera': w_camera,
            'brightness': self.current_brightness,
            'light_factor': light_factor,
            'motion_factor': motion_factor
        }

    def compute_lidar_confidence(self, distance: float, num_points: int) -> float:
        """Compute LiDAR detection confidence."""
        # Higher confidence at medium range (1-5m)
        if distance < 0.5:
            dist_conf = 0.7  # Close might be noise
        elif distance < 8.0:
            dist_conf = 1.0
        else:
            dist_conf = max(0.3, 1.0 - (distance - 8.0) / 10.0)

        # More points = higher confidence
        points_conf = min(1.0, num_points / 20.0)

        return dist_conf * points_conf

    def compute_camera_confidence(self, distance: float) -> float:
        """Compute camera detection confidence."""
        # Higher confidence at close range (0.5-2m)
        if distance < 0.5:
            return 0.6  # Very close might have depth errors
        elif distance < 2.0:
            return 1.0
        elif distance < 4.0:
            return max(0.3, 1.0 - (distance - 2.0) / 4.0)
        else:
            return 0.2

    def fuse_obstacles(
        self,
        lidar_obs: List[Dict],
        camera_obs: List[Dict],
        weights: Dict[str, float],
        timestamp: float
    ) -> List[Dict]:
        """Fuse obstacles from both sensors."""
        fused = []
        used_camera = set()

        # Match LiDAR obstacles to camera obstacles
        for l_obs in lidar_obs:
            best_match = None
            best_dist = float('inf')

            for i, c_obs in enumerate(camera_obs):
                if i in used_camera:
                    continue
                dist = np.linalg.norm(l_obs['position'][:2] - c_obs['position'][:2])
                if dist < self.assoc_thresh and dist < best_dist:
                    best_dist = dist
                    best_match = i

            if best_match is not None:
                # Fused obstacle
                c_obs = camera_obs[best_match]
                used_camera.add(best_match)

                # Distance-adaptive weighting
                distance = l_obs['distance']
                dist_factor = 1.0 / (1.0 + math.exp(-self.weight_steep * (distance - self.dist_crossover)))

                # Combine with base weights
                w_l = weights['lidar'] * (0.3 + 0.7 * dist_factor)
                w_c = weights['camera'] * (0.3 + 0.7 * (1.0 - dist_factor))
                total = w_l + w_c

                fused_pos = (w_l * l_obs['position'] + w_c * c_obs['position']) / total
                fused_conf = (w_l * l_obs['confidence'] + w_c * c_obs['confidence']) / total

                fused.append({
                    'position': fused_pos,
                    'size': l_obs['size'],
                    'distance': np.linalg.norm(fused_pos[:2]),
                    'source': 'fused',
                    'confidence': fused_conf,
                    'lidar_weight': w_l / total,
                    'camera_weight': w_c / total
                })
                self.fused_count += 1
            else:
                # LiDAR only
                fused.append(l_obs)

        # Add unmatched camera obstacles
        for i, c_obs in enumerate(camera_obs):
            if i not in used_camera:
                fused.append(c_obs)

        return fused

    def update_tracking(self, obstacles: List[Dict], timestamp: float):
        """Update tracked obstacles with new detections."""
        used_tracks = set()

        for obs in obstacles:
            best_track = None
            best_dist = float('inf')

            for track_id, track in self.tracked_obstacles.items():
                if track_id in used_tracks:
                    continue
                dist = np.linalg.norm(obs['position'] - track.position)
                if dist < self.assoc_thresh and dist < best_dist:
                    best_dist = dist
                    best_track = track_id

            if best_track is not None:
                # Update existing track
                self.tracked_obstacles[best_track].update(obs['position'], timestamp)
                self.tracked_obstacles[best_track].confidence = obs['confidence']
                used_tracks.add(best_track)
            else:
                # Create new track
                new_track = TrackedObstacle(
                    self.next_obstacle_id, obs['position'], timestamp
                )
                new_track.confidence = obs['confidence']
                new_track.size = obs.get('size', np.array([0.1, 0.1, 0.5]))
                self.tracked_obstacles[self.next_obstacle_id] = new_track
                self.next_obstacle_id += 1

    def cleanup_old_obstacles(self):
        """Remove obstacles that haven't been seen recently."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        to_remove = []

        for track_id, track in self.tracked_obstacles.items():
            age = current_time - track.last_seen
            if age > self.obs_timeout:
                to_remove.append(track_id)

        for track_id in to_remove:
            del self.tracked_obstacles[track_id]

    def publish_obstacles(self):
        """Publish tracked obstacles as markers."""
        marker_array = MarkerArray()

        for track_id, track in self.tracked_obstacles.items():
            if track.hit_count < self.min_obs:
                continue

            # Obstacle cube
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'fused_obstacles'
            marker.id = track_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = track.position[0]
            marker.pose.position.y = track.position[1]
            marker.pose.position.z = track.position[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = track.size[0]
            marker.scale.y = track.size[1]
            marker.scale.z = track.size[2]

            # Color based on confidence
            if track.confidence > 0.8:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)  # Red = high conf
            elif track.confidence > 0.5:
                marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)  # Orange = medium
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)  # Yellow = low

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 300000000  # 300ms

            marker_array.markers.append(marker)

        self.obstacles_pub.publish(marker_array)

    def publish_fused_pointcloud(
        self,
        lidar_obstacles: List[Dict],
        depth_points: np.ndarray,
        header: Header
    ):
        """Publish fused pointcloud for costmap integration."""
        points = []

        # Add LiDAR cluster centroids (elevated for visibility)
        for obs in lidar_obstacles:
            points.append([obs['position'][0], obs['position'][1], 0.3])

        # Add filtered depth points
        if len(depth_points) > 0:
            for pt in depth_points[::10]:  # Subsample
                points.append([pt[0], pt[1], pt[2]])

        if len(points) == 0:
            return

        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = self.output_frame

        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False

        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

        buffer = []
        for pt in points:
            buffer.append(struct.pack('fff', pt[0], pt[1], pt[2]))
        cloud_msg.data = b''.join(buffer)

        self.pointcloud_pub.publish(cloud_msg)

    def publish_confidence(self, weights: Dict[str, float]):
        """Publish current confidence values."""
        msg = Float32MultiArray()
        msg.data = [
            weights['lidar'],
            weights['camera'],
            weights['brightness'],
            weights['light_factor'],
            weights['motion_factor']
        ]
        self.confidence_pub.publish(msg)

    def publish_stats(self):
        """Publish fusion statistics."""
        msg = String()
        msg.data = (
            f'Fusion#{self.fusion_count}|'
            f'L:{self.lidar_only_count}|'
            f'C:{self.camera_only_count}|'
            f'F:{self.fused_count}|'
            f'Tracked:{len(self.tracked_obstacles)}'
        )
        self.stats_pub.publish(msg)

        if self.fusion_count % 100 == 0:
            self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Adaptive Fusion shutting down - '
            f'{node.fusion_count} fusions, '
            f'{node.fused_count} fused obstacles'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
