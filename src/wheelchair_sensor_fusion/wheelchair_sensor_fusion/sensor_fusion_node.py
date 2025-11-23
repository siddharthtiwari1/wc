#!/usr/bin/env python3
"""Main Sensor Fusion Node for Wheelchair Obstacle Avoidance.

This node performs adaptive fusion of 2D LiDAR clusters and camera-based
YOLO detections to create a unified obstacle representation.

Implements the novel Adaptive Semantic-Geometric Fusion Algorithm described
in the accompanying research paper.

Author: Siddharth Tiwari
Email: s24035@students.iitmandi.ac.in
"""

import numpy as np
import math
from typing import List, Dict, Optional, Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import message_filters
import cv2
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException
import json


class FusedObstacle:
    """Represents a fused obstacle combining LiDAR and camera data."""

    def __init__(self, obstacle_id: int):
        """Initialize fused obstacle.

        Args:
            obstacle_id: Unique identifier for this obstacle
        """
        self.id = obstacle_id
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z in base_link
        self.size = np.array([0.0, 0.0, 0.0])  # width, height, depth
        self.class_name = 'unknown'
        self.confidence = 0.0
        self.lidar_weight = 0.0
        self.camera_weight = 0.0
        self.distance = 0.0
        self.has_lidar = False
        self.has_camera = False

    def to_dict(self) -> dict:
        """Convert to dictionary for logging."""
        return {
            'id': self.id,
            'position': self.position.tolist(),
            'size': self.size.tolist(),
            'class': self.class_name,
            'confidence': self.confidence,
            'distance': self.distance,
            'lidar_weight': self.lidar_weight,
            'camera_weight': self.camera_weight
        }


class SensorFusionNode(Node):
    """Adaptive sensor fusion combining LiDAR and camera detections."""

    def __init__(self):
        """Initialize the sensor fusion node."""
        super().__init__('sensor_fusion_node')

        # Declare parameters
        self.declare_parameter('lidar_clusters_topic', '/lidar/clusters')
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('fused_obstacles_topic', '/fusion/obstacles')
        self.declare_parameter('visualization_topic', '/fusion/visualization')

        # Fusion parameters
        self.declare_parameter('fusion_iou_threshold', 0.3)
        self.declare_parameter('max_association_distance', 0.5)
        self.declare_parameter('adaptive_weighting', True)
        self.declare_parameter('distance_threshold', 2.0)  # meters
        self.declare_parameter('lighting_adaptive', True)

        # TF parameters
        self.declare_parameter('lidar_frame', 'lidar')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        # Synchronization
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.05)  # 50ms tolerance

        # Get parameters
        self.lidar_topic = self.get_parameter('lidar_clusters_topic').value
        self.yolo_topic = self.get_parameter('yolo_detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_image_topic').value
        self.fused_topic = self.get_parameter('fused_obstacles_topic').value
        self.viz_topic = self.get_parameter('visualization_topic').value

        self.iou_threshold = self.get_parameter('fusion_iou_threshold').value
        self.max_assoc_dist = self.get_parameter('max_association_distance').value
        self.adaptive_weight = self.get_parameter('adaptive_weighting').value
        self.dist_threshold = self.get_parameter('distance_threshold').value
        self.lighting_adapt = self.get_parameter('lighting_adaptive').value

        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        queue_size = self.get_parameter('sync_queue_size').value
        slop = self.get_parameter('sync_slop').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera calibration
        self.camera_info = None
        self.K = None  # Intrinsic matrix
        self.D = None  # Distortion coefficients

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Transforms
        self.T_lidar_to_camera = None
        self.T_camera_to_base = None

        # Publishers
        self.fused_pub = self.create_publisher(
            MarkerArray,
            self.fused_topic,
            10
        )
        self.viz_pub = self.create_publisher(
            MarkerArray,
            self.viz_topic,
            10
        )

        # Camera info subscriber (for calibration)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Synchronized subscribers
        self.lidar_sub = message_filters.Subscriber(
            self,
            MarkerArray,
            self.lidar_topic
        )
        self.yolo_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            self.yolo_topic
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic
        )

        # Time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.yolo_sub, self.depth_sub],
            queue_size=queue_size,
            slop=slop
        )
        self.sync.registerCallback(self.fusion_callback)

        # Statistics
        self.fusion_count = 0
        self.obstacle_id_counter = 0

        self.get_logger().info('Sensor Fusion Node initialized')
        self.get_logger().info(f'  LiDAR topic: {self.lidar_topic}')
        self.get_logger().info(f'  YOLO topic: {self.yolo_topic}')
        self.get_logger().info(f'  Depth topic: {self.depth_topic}')
        self.get_logger().info(f'  Adaptive weighting: {self.adaptive_weight}')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration parameters.

        Args:
            msg: CameraInfo message
        """
        if self.camera_info is None:
            self.camera_info = msg
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d)
            self.get_logger().info('Camera calibration received')

    def fusion_callback(
        self,
        lidar_msg: MarkerArray,
        yolo_msg: Detection2DArray,
        depth_msg: Image
    ):
        """Main fusion callback - synchronized processing.

        Args:
            lidar_msg: LiDAR cluster markers
            yolo_msg: YOLO detections
            depth_msg: Aligned depth image
        """
        self.fusion_count += 1

        # Update transforms
        self.update_transforms()

        if self.K is None:
            self.get_logger().warn('No camera calibration available yet')
            return

        # Convert depth image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        # Extract LiDAR clusters
        lidar_clusters = self.extract_lidar_clusters(lidar_msg)

        # Extract YOLO detections
        yolo_detections = yolo_msg.detections

        # Perform fusion
        fused_obstacles = self.fuse_sensors(
            lidar_clusters,
            yolo_detections,
            depth_image,
            yolo_msg.header
        )

        # Publish fused obstacles
        if fused_obstacles:
            obstacle_markers = self.create_obstacle_markers(
                fused_obstacles,
                yolo_msg.header
            )
            self.fused_pub.publish(obstacle_markers)

            # Publish visualization
            viz_markers = self.create_visualization_markers(
                fused_obstacles,
                yolo_msg.header
            )
            self.viz_pub.publish(viz_markers)

        # Log statistics
        if self.fusion_count % 50 == 0:
            self.get_logger().info(
                f'Fusion #{self.fusion_count}: '
                f'{len(lidar_clusters)} LiDAR clusters, '
                f'{len(yolo_detections)} YOLO detections, '
                f'{len(fused_obstacles)} fused obstacles'
            )

    def update_transforms(self):
        """Update TF transforms between frames."""
        try:
            # Get transform from LiDAR to camera
            if self.T_lidar_to_camera is None:
                t = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.lidar_frame,
                    rclpy.time.Time()
                )
                self.T_lidar_to_camera = self.transform_to_matrix(t)
                self.get_logger().info('LiDAR-to-camera transform obtained')

            # Get transform from camera to base_link
            if self.T_camera_to_base is None:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    rclpy.time.Time()
                )
                self.T_camera_to_base = self.transform_to_matrix(t)
                self.get_logger().info('Camera-to-base transform obtained')

        except TransformException as ex:
            pass  # Silently wait for transforms

    def transform_to_matrix(self, transform) -> np.ndarray:
        """Convert ROS Transform to 4x4 transformation matrix.

        Args:
            transform: ROS TransformStamped message

        Returns:
            4x4 numpy transformation matrix
        """
        t = transform.transform
        q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]

        # Quaternion to rotation matrix
        R = self.quaternion_to_rotation_matrix(q)

        # Create 4x4 matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]

        return T

    def quaternion_to_rotation_matrix(self, q: list) -> np.ndarray:
        """Convert quaternion to rotation matrix.

        Args:
            q: Quaternion [x, y, z, w]

        Returns:
            3x3 rotation matrix
        """
        x, y, z, w = q
        R = np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
        return R

    def extract_lidar_clusters(self, marker_array: MarkerArray) -> List[dict]:
        """Extract cluster information from LiDAR markers.

        Args:
            marker_array: MarkerArray containing cluster centroids and bboxes

        Returns:
            List of cluster dictionaries
        """
        clusters = []
        centroids = {}
        bboxes = {}

        # Separate centroids and bboxes
        for marker in marker_array.markers:
            if marker.ns == 'cluster_centroids':
                centroids[marker.id] = marker
            elif marker.ns == 'cluster_bboxes':
                bboxes[marker.id] = marker

        # Combine centroid + bbox info
        for cluster_id in centroids:
            if cluster_id in bboxes:
                centroid_marker = centroids[cluster_id]
                bbox_marker = bboxes[cluster_id]

                cluster = {
                    'id': cluster_id,
                    'position': np.array([
                        centroid_marker.pose.position.x,
                        centroid_marker.pose.position.y,
                        centroid_marker.pose.position.z
                    ]),
                    'size': np.array([
                        bbox_marker.scale.x,
                        bbox_marker.scale.y,
                        bbox_marker.scale.z
                    ])
                }
                clusters.append(cluster)

        return clusters

    def fuse_sensors(
        self,
        lidar_clusters: List[dict],
        yolo_detections: List,
        depth_image: np.ndarray,
        header: Header
    ) -> List[FusedObstacle]:
        """Main fusion algorithm combining LiDAR and camera data.

        Args:
            lidar_clusters: List of LiDAR cluster dicts
            yolo_detections: List of YOLO Detection2D messages
            depth_image: Depth image from RealSense
            header: Message header

        Returns:
            List of fused obstacles
        """
        fused_obstacles = []

        # Create association matrix
        association_matrix = self.compute_association_scores(
            lidar_clusters,
            yolo_detections,
            depth_image
        )

        # Match LiDAR clusters to YOLO detections
        matched_pairs = self.match_detections(association_matrix)

        # Create fused obstacles from matched pairs
        for lidar_idx, yolo_idx in matched_pairs:
            obstacle = self.create_fused_obstacle(
                lidar_clusters[lidar_idx],
                yolo_detections[yolo_idx],
                depth_image
            )
            if obstacle:
                fused_obstacles.append(obstacle)

        # Add unmatched LiDAR clusters
        matched_lidar = set(pair[0] for pair in matched_pairs)
        for i, cluster in enumerate(lidar_clusters):
            if i not in matched_lidar:
                obstacle = self.create_lidar_only_obstacle(cluster)
                fused_obstacles.append(obstacle)

        # Add unmatched YOLO detections (camera-only)
        matched_yolo = set(pair[1] for pair in matched_pairs)
        for i, detection in enumerate(yolo_detections):
            if i not in matched_yolo:
                obstacle = self.create_camera_only_obstacle(
                    detection,
                    depth_image
                )
                if obstacle:
                    fused_obstacles.append(obstacle)

        return fused_obstacles

    def compute_association_scores(
        self,
        lidar_clusters: List[dict],
        yolo_detections: List,
        depth_image: np.ndarray
    ) -> np.ndarray:
        """Compute association scores between LiDAR and camera detections.

        Args:
            lidar_clusters: LiDAR clusters
            yolo_detections: YOLO detections
            depth_image: Depth image

        Returns:
            NxM matrix of association scores
        """
        n_lidar = len(lidar_clusters)
        n_yolo = len(yolo_detections)

        if n_lidar == 0 or n_yolo == 0:
            return np.array([])

        scores = np.zeros((n_lidar, n_yolo))

        for i, cluster in enumerate(lidar_clusters):
            # Project LiDAR point to image
            img_point = self.project_lidar_to_image(cluster['position'])
            if img_point is None:
                continue

            for j, detection in enumerate(yolo_detections):
                # Get bounding box
                bbox = self.get_bbox_from_detection(detection)

                # Check if projection falls inside bbox
                if self.point_in_bbox(img_point, bbox):
                    # Compute IoU-like score
                    confidence = detection.results[0].hypothesis.score
                    scores[i, j] = confidence
                else:
                    scores[i, j] = 0.0

        return scores

    def project_lidar_to_image(
        self,
        point_lidar: np.ndarray
    ) -> Optional[Tuple[int, int]]:
        """Project 3D LiDAR point to 2D image coordinates.

        Args:
            point_lidar: 3D point in LiDAR frame

        Returns:
            (u, v) image coordinates, or None if behind camera
        """
        if self.T_lidar_to_camera is None:
            return None

        # Transform to camera frame
        point_hom = np.append(point_lidar, 1.0)
        point_camera = self.T_lidar_to_camera @ point_hom

        # Check if point is in front of camera
        if point_camera[2] <= 0:
            return None

        # Project to image
        point_3d = point_camera[:3]
        uv_hom = self.K @ point_3d
        u = int(uv_hom[0] / uv_hom[2])
        v = int(uv_hom[1] / uv_hom[2])

        return (u, v)

    def get_bbox_from_detection(self, detection) -> Tuple[int, int, int, int]:
        """Extract bounding box from Detection2D message.

        Args:
            detection: Detection2D message

        Returns:
            (x1, y1, x2, y2) bounding box coordinates
        """
        center_x = detection.bbox.center.position.x
        center_y = detection.bbox.center.position.y
        size_x = detection.bbox.size_x
        size_y = detection.bbox.size_y

        x1 = int(center_x - size_x / 2)
        y1 = int(center_y - size_y / 2)
        x2 = int(center_x + size_x / 2)
        y2 = int(center_y + size_y / 2)

        return (x1, y1, x2, y2)

    def point_in_bbox(
        self,
        point: Tuple[int, int],
        bbox: Tuple[int, int, int, int]
    ) -> bool:
        """Check if point is inside bounding box.

        Args:
            point: (u, v) image coordinates
            bbox: (x1, y1, x2, y2) bounding box

        Returns:
            True if point is inside bbox
        """
        u, v = point
        x1, y1, x2, y2 = bbox
        return x1 <= u <= x2 and y1 <= v <= y2

    def match_detections(
        self,
        scores: np.ndarray
    ) -> List[Tuple[int, int]]:
        """Match LiDAR clusters to YOLO detections using Hungarian algorithm.

        Args:
            scores: NxM association score matrix

        Returns:
            List of (lidar_idx, yolo_idx) pairs
        """
        if scores.size == 0:
            return []

        # Simple greedy matching (can be replaced with Hungarian algorithm)
        matched_pairs = []
        used_lidar = set()
        used_yolo = set()

        # Sort by score (descending)
        indices = np.dstack(np.unravel_index(
            np.argsort(scores.ravel())[::-1],
            scores.shape
        ))[0]

        for i, j in indices:
            if scores[i, j] > 0 and i not in used_lidar and j not in used_yolo:
                matched_pairs.append((i, j))
                used_lidar.add(i)
                used_yolo.add(j)

        return matched_pairs

    def create_fused_obstacle(
        self,
        lidar_cluster: dict,
        yolo_detection,
        depth_image: np.ndarray
    ) -> Optional[FusedObstacle]:
        """Create fused obstacle from matched LiDAR and camera data.

        Args:
            lidar_cluster: LiDAR cluster dict
            yolo_detection: YOLO Detection2D message
            depth_image: Depth image

        Returns:
            FusedObstacle or None
        """
        obstacle = FusedObstacle(self.obstacle_id_counter)
        self.obstacle_id_counter += 1

        # Get LiDAR position
        lidar_pos = lidar_cluster['position']
        lidar_dist = np.linalg.norm(lidar_pos[:2])

        # Get camera depth
        bbox = self.get_bbox_from_detection(yolo_detection)
        camera_depth = self.get_median_depth(depth_image, bbox)

        # Adaptive weighting
        if self.adaptive_weight:
            w_lidar, w_camera = self.compute_adaptive_weights(
                lidar_dist,
                camera_depth,
                yolo_detection.results[0].hypothesis.score
            )
        else:
            w_lidar = 0.5
            w_camera = 0.5

        # Fuse positions
        # For 2D LiDAR, use LiDAR XY and camera depth for Z
        obstacle.position[0] = lidar_pos[0]  # X from LiDAR
        obstacle.position[1] = lidar_pos[1]  # Y from LiDAR
        obstacle.position[2] = 0.0  # 2D LiDAR

        # Fuse size
        obstacle.size = lidar_cluster['size']

        # Get semantic information from YOLO
        obstacle.class_name = yolo_detection.id  # Class name stored in id field
        obstacle.confidence = yolo_detection.results[0].hypothesis.score

        # Store weights
        obstacle.lidar_weight = w_lidar
        obstacle.camera_weight = w_camera
        obstacle.distance = lidar_dist
        obstacle.has_lidar = True
        obstacle.has_camera = True

        return obstacle

    def create_lidar_only_obstacle(self, cluster: dict) -> FusedObstacle:
        """Create obstacle from LiDAR data only.

        Args:
            cluster: LiDAR cluster dict

        Returns:
            FusedObstacle
        """
        obstacle = FusedObstacle(self.obstacle_id_counter)
        self.obstacle_id_counter += 1

        obstacle.position = cluster['position']
        obstacle.size = cluster['size']
        obstacle.class_name = 'obstacle'
        obstacle.confidence = 0.7
        obstacle.lidar_weight = 1.0
        obstacle.camera_weight = 0.0
        obstacle.distance = np.linalg.norm(cluster['position'][:2])
        obstacle.has_lidar = True
        obstacle.has_camera = False

        return obstacle

    def create_camera_only_obstacle(
        self,
        detection,
        depth_image: np.ndarray
    ) -> Optional[FusedObstacle]:
        """Create obstacle from camera detection only.

        Args:
            detection: YOLO Detection2D message
            depth_image: Depth image

        Returns:
            FusedObstacle or None
        """
        bbox = self.get_bbox_from_detection(detection)
        depth = self.get_median_depth(depth_image, bbox)

        if depth == 0:
            return None  # No valid depth

        # Compute 3D position from depth
        center_x = detection.bbox.center.position.x
        center_y = detection.bbox.center.position.y

        # Backproject to 3D
        point_3d = self.backproject_pixel(center_x, center_y, depth)
        if point_3d is None:
            return None

        obstacle = FusedObstacle(self.obstacle_id_counter)
        self.obstacle_id_counter += 1

        obstacle.position = point_3d
        obstacle.size = np.array([0.3, 0.3, 0.5])  # Default size
        obstacle.class_name = detection.id
        obstacle.confidence = detection.results[0].hypothesis.score
        obstacle.lidar_weight = 0.0
        obstacle.camera_weight = 1.0
        obstacle.distance = np.linalg.norm(point_3d[:2])
        obstacle.has_lidar = False
        obstacle.has_camera = True

        return obstacle

    def get_median_depth(
        self,
        depth_image: np.ndarray,
        bbox: Tuple[int, int, int, int]
    ) -> float:
        """Get median depth in bounding box region.

        Args:
            depth_image: Depth image (mm)
            bbox: (x1, y1, x2, y2)

        Returns:
            Median depth in meters
        """
        x1, y1, x2, y2 = bbox
        h, w = depth_image.shape

        # Clip to image bounds
        x1 = max(0, min(x1, w-1))
        x2 = max(0, min(x2, w-1))
        y1 = max(0, min(y1, h-1))
        y2 = max(0, min(y2, h-1))

        if x2 <= x1 or y2 <= y1:
            return 0.0

        roi = depth_image[y1:y2, x1:x2]
        valid_depths = roi[roi > 0]

        if len(valid_depths) == 0:
            return 0.0

        # Convert from mm to meters
        median_depth = np.median(valid_depths) / 1000.0
        return float(median_depth)

    def backproject_pixel(
        self,
        u: float,
        v: float,
        depth: float
    ) -> Optional[np.ndarray]:
        """Backproject pixel to 3D point in camera frame.

        Args:
            u, v: Pixel coordinates
            depth: Depth in meters

        Returns:
            3D point [x, y, z] or None
        """
        if depth <= 0:
            return None

        # Inverse projection
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return np.array([x, y, z])

    def compute_adaptive_weights(
        self,
        lidar_dist: float,
        camera_depth: float,
        yolo_confidence: float
    ) -> Tuple[float, float]:
        """Compute adaptive fusion weights based on sensor reliability.

        Args:
            lidar_dist: Distance from LiDAR
            camera_depth: Depth from camera
            yolo_confidence: YOLO detection confidence

        Returns:
            (w_lidar, w_camera) weights
        """
        # Favor LiDAR at longer distances (sigmoid function)
        k = 2.0  # Steepness
        dist_weight = 1.0 / (1.0 + math.exp(-k * (lidar_dist - self.dist_threshold)))

        # Consider YOLO confidence
        conf_weight = yolo_confidence

        # Combine factors
        w_lidar = 0.3 + 0.7 * dist_weight  # Range: 0.3 to 1.0
        w_camera = 0.3 + 0.7 * (1.0 - dist_weight) * conf_weight

        # Normalize
        total = w_lidar + w_camera
        if total > 0:
            w_lidar /= total
            w_camera /= total
        else:
            w_lidar = 0.5
            w_camera = 0.5

        return (w_lidar, w_camera)

    def create_obstacle_markers(
        self,
        obstacles: List[FusedObstacle],
        header: Header
    ) -> MarkerArray:
        """Create marker array for fused obstacles.

        Args:
            obstacles: List of fused obstacles
            header: Message header

        Returns:
            MarkerArray
        """
        marker_array = MarkerArray()

        for obs in obstacles:
            # Create obstacle marker
            marker = Marker()
            marker.header = header
            marker.ns = 'fused_obstacles'
            marker.id = obs.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obs.position[0]
            marker.pose.position.y = obs.position[1]
            marker.pose.position.z = obs.position[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(obs.size[0], 0.1)
            marker.scale.y = max(obs.size[1], 0.1)
            marker.scale.z = max(obs.size[2], 0.3)

            # Color based on source
            if obs.has_lidar and obs.has_camera:
                # Purple for fused
                marker.color = ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.7)
            elif obs.has_lidar:
                # Green for LiDAR only
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
            else:
                # Blue for camera only
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 0.2 seconds

            marker_array.markers.append(marker)

        return marker_array

    def create_visualization_markers(
        self,
        obstacles: List[FusedObstacle],
        header: Header
    ) -> MarkerArray:
        """Create detailed visualization markers.

        Args:
            obstacles: List of fused obstacles
            header: Message header

        Returns:
            MarkerArray
        """
        marker_array = MarkerArray()

        for obs in obstacles:
            # Text label
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = 'obstacle_labels'
            text_marker.id = obs.id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obs.position[0]
            text_marker.pose.position.y = obs.position[1]
            text_marker.pose.position.z = obs.position[2] + 0.3
            text_marker.scale.z = 0.15
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f'{obs.class_name}\n{obs.distance:.2f}m\nC:{obs.confidence:.2f}'
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 200000000

            marker_array.markers.append(text_marker)

        return marker_array


def main(args=None):
    """Run the sensor fusion node."""
    rclpy.init(args=args)
    node = SensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down - Processed {node.fusion_count} fusion cycles'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
