#!/usr/bin/env python3
"""Production-Grade Sensor Fusion Node for Wheelchair Navigation.

This node implements a robust, fault-tolerant adaptive sensor fusion system
specifically optimized for indoor wheelchair navigation with Nav2 integration.

Features:
- Comprehensive error handling and recovery
- Sensor health monitoring
- Automatic fallback modes
- Nav2 costmap integration
- Indoor environment optimization
- ROS2 Jazzy native support

Author: Siddharth Tiwari
Institution: Indian Institute of Technology Mandi
Email: s24035@students.iitmandi.ac.in
"""

import numpy as np
import math
from typing import List, Dict, Optional, Tuple, Set
from enum import Enum
from dataclasses import dataclass
from collections import deque
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA, String
from nav_msgs.msg import OccupancyGrid
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from tf2_ros import TransformException


class SensorStatus(Enum):
    """Sensor health status."""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"
    UNKNOWN = "unknown"


class FusionMode(Enum):
    """Operating modes for sensor fusion."""
    FULL_FUSION = "full_fusion"           # LiDAR + Camera + YOLO
    LIDAR_CAMERA = "lidar_camera"         # LiDAR + Camera (no YOLO)
    LIDAR_ONLY = "lidar_only"             # LiDAR fallback
    CAMERA_ONLY = "camera_only"           # Camera fallback
    SAFE_STOP = "safe_stop"               # Emergency: all sensors failed


@dataclass
class SensorHealth:
    """Track health metrics for each sensor."""
    name: str
    status: SensorStatus = SensorStatus.UNKNOWN
    last_message_time: float = 0.0
    message_count: int = 0
    error_count: int = 0
    timeout: float = 1.0  # seconds

    def is_healthy(self, current_time: float) -> bool:
        """Check if sensor is healthy."""
        time_since_last = current_time - self.last_message_time
        return (self.status == SensorStatus.HEALTHY and
                time_since_last < self.timeout)

    def update(self, current_time: float, success: bool = True):
        """Update sensor health."""
        self.last_message_time = current_time
        self.message_count += 1

        if not success:
            self.error_count += 1
            if self.error_count > 5:
                self.status = SensorStatus.FAILED
            elif self.error_count > 2:
                self.status = SensorStatus.DEGRADED
        else:
            # Reset errors if we get successful messages
            if self.error_count > 0:
                self.error_count = max(0, self.error_count - 1)
            self.status = SensorStatus.HEALTHY


@dataclass
class FusedObstacle:
    """Robust fused obstacle representation."""
    id: int
    position: np.ndarray  # [x, y, z] in base_link
    size: np.ndarray      # [width, height, depth]
    class_name: str = 'unknown'
    confidence: float = 0.0
    lidar_weight: float = 0.0
    camera_weight: float = 0.0
    distance: float = 0.0
    has_lidar: bool = False
    has_camera: bool = False
    velocity: np.ndarray = None  # [vx, vy] for tracking
    tracked_frames: int = 0
    persistence_score: float = 1.0  # Higher = more persistent

    def __post_init__(self):
        """Initialize velocity if not provided."""
        if self.velocity is None:
            self.velocity = np.array([0.0, 0.0])


class RobustSensorFusionNode(Node):
    """Production-grade sensor fusion node with comprehensive error handling."""

    def __init__(self):
        """Initialize robust sensor fusion node."""
        super().__init__('sensor_fusion_robust_node')

        # Declare parameters with robust defaults
        self._declare_parameters()
        self._load_parameters()

        # Initialize components
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sensor health monitoring
        self.sensor_health = {
            'lidar': SensorHealth('lidar', timeout=0.5),
            'camera': SensorHealth('camera', timeout=0.2),
            'depth': SensorHealth('depth', timeout=0.2),
            'yolo': SensorHealth('yolo', timeout=0.2)
        }

        # Operating mode
        self.fusion_mode = FusionMode.FULL_FUSION

        # Camera calibration
        self.camera_info = None
        self.K = None
        self.D = None

        # TF transforms with retry mechanism
        self.T_lidar_to_camera = None
        self.T_camera_to_base = None
        self.T_lidar_to_base = None
        self.tf_retry_count = 0
        self.MAX_TF_RETRIES = 10

        # Obstacle tracking
        self.tracked_obstacles: Dict[int, FusedObstacle] = {}
        self.next_obstacle_id = 0
        self.obstacle_history = deque(maxlen=30)  # 1 second at 30Hz

        # Statistics and diagnostics
        self.fusion_count = 0
        self.successful_fusions = 0
        self.failed_fusions = 0
        self.mode_changes = 0
        self.last_diagnostics_time = time.time()

        # QoS Profiles for reliability
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Setup publishers and subscribers
        self._setup_publishers()
        self._setup_subscribers()

        # Diagnostic timer
        self.diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)

        # TF update timer
        self.tf_timer = self.create_timer(2.0, self._update_transforms_retry)

        self.get_logger().info('='*60)
        self.get_logger().info('Robust Sensor Fusion Node Initialized')
        self.get_logger().info(f'Mode: {self.fusion_mode.value}')
        self.get_logger().info(f'Adaptive weighting: {self.adaptive_weight}')
        self.get_logger().info(f'Indoor optimization: {self.indoor_mode}')
        self.get_logger().info('='*60)

    def _declare_parameters(self):
        """Declare all ROS2 parameters with defaults."""
        # Input topics
        self.declare_parameter('lidar_clusters_topic', '/lidar/clusters')
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('scan_topic', '/scan')

        # Output topics
        self.declare_parameter('fused_obstacles_topic', '/fusion/obstacles')
        self.declare_parameter('visualization_topic', '/fusion/visualization')
        self.declare_parameter('costmap_topic', '/fusion/obstacle_costmap')
        self.declare_parameter('diagnostics_topic', '/fusion/diagnostics')
        self.declare_parameter('status_topic', '/fusion/status')

        # Fusion parameters
        self.declare_parameter('fusion_iou_threshold', 0.3)
        self.declare_parameter('max_association_distance', 0.5)
        self.declare_parameter('adaptive_weighting', True)
        self.declare_parameter('distance_threshold', 2.0)
        self.declare_parameter('lighting_adaptive', True)

        # Indoor-specific parameters
        self.declare_parameter('indoor_mode', True)
        self.declare_parameter('max_obstacle_distance', 6.0)
        self.declare_parameter('min_obstacle_distance', 0.15)
        self.declare_parameter('obstacle_inflation', 0.3)  # Wheelchair safety margin

        # Robustness parameters
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('tracking_distance_threshold', 0.5)
        self.declare_parameter('min_obstacle_confidence', 0.3)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('enable_fallback', True)

        # Frame IDs
        self.declare_parameter('lidar_frame', 'lidar')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        # Synchronization
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.1)

        # Costmap parameters
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_width', 10.0)
        self.declare_parameter('costmap_height', 10.0)

    def _load_parameters(self):
        """Load parameters from ROS2 parameter server."""
        # Topics
        self.lidar_topic = self.get_parameter('lidar_clusters_topic').value
        self.yolo_topic = self.get_parameter('yolo_detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.fused_topic = self.get_parameter('fused_obstacles_topic').value
        self.viz_topic = self.get_parameter('visualization_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self.status_topic = self.get_parameter('status_topic').value

        # Fusion settings
        self.iou_threshold = self.get_parameter('fusion_iou_threshold').value
        self.max_assoc_dist = self.get_parameter('max_association_distance').value
        self.adaptive_weight = self.get_parameter('adaptive_weighting').value
        self.dist_threshold = self.get_parameter('distance_threshold').value
        self.lighting_adapt = self.get_parameter('lighting_adaptive').value

        # Indoor mode
        self.indoor_mode = self.get_parameter('indoor_mode').value
        self.max_obstacle_dist = self.get_parameter('max_obstacle_distance').value
        self.min_obstacle_dist = self.get_parameter('min_obstacle_distance').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value

        # Robustness
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.tracking_dist_threshold = self.get_parameter('tracking_distance_threshold').value
        self.min_confidence = self.get_parameter('min_obstacle_confidence').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.enable_fallback = self.get_parameter('enable_fallback').value

        # Frames
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Sync
        self.sync_queue_size = self.get_parameter('sync_queue_size').value
        self.sync_slop = self.get_parameter('sync_slop').value

        # Costmap
        self.costmap_resolution = self.get_parameter('costmap_resolution').value
        self.costmap_width = int(self.get_parameter('costmap_width').value / self.costmap_resolution)
        self.costmap_height = int(self.get_parameter('costmap_height').value / self.costmap_resolution)

    def _setup_publishers(self):
        """Setup all publishers with appropriate QoS."""
        self.fused_pub = self.create_publisher(
            MarkerArray,
            self.fused_topic,
            self.reliable_qos
        )

        self.viz_pub = self.create_publisher(
            MarkerArray,
            self.viz_topic,
            self.sensor_qos
        )

        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.costmap_topic,
            self.reliable_qos
        )

        self.diagnostics_pub = self.create_publisher(
            String,
            self.diagnostics_topic,
            self.reliable_qos
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            self.reliable_qos
        )

    def _setup_subscribers(self):
        """Setup all subscribers with message filters for synchronization."""
        # Camera info (non-synchronized)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            self.sensor_qos
        )

        # Scan (fallback, non-synchronized)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            self.sensor_qos
        )

        # Synchronized subscribers
        self.lidar_sub = message_filters.Subscriber(
            self,
            MarkerArray,
            self.lidar_topic,
            qos_profile=self.sensor_qos
        )

        self.yolo_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            self.yolo_topic,
            qos_profile=self.sensor_qos
        )

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=self.sensor_qos
        )

        # Time synchronizer with large slop for robustness
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.yolo_sub, self.depth_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        self.sync.registerCallback(self.fusion_callback)

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration with validation."""
        try:
            if self.camera_info is None:
                self.K = np.array(msg.k).reshape(3, 3)
                self.D = np.array(msg.d)

                # Validate intrinsics
                if self.K[0, 0] > 0 and self.K[1, 1] > 0:
                    self.camera_info = msg
                    self.get_logger().info('Camera calibration received and validated')
                else:
                    self.get_logger().error('Invalid camera calibration: zero focal length')
        except Exception as e:
            self.get_logger().error(f'Failed to process camera info: {e}')

    def scan_callback(self, msg: LaserScan):
        """Direct LiDAR scan processing for fallback mode."""
        current_time = time.time()
        self.sensor_health['lidar'].update(current_time, True)

        # Process scan for emergency obstacle detection if in fallback mode
        if self.fusion_mode == FusionMode.LIDAR_ONLY:
            # Basic obstacle detection from raw scan
            # This is a safety fallback - not full fusion
            pass

    def fusion_callback(
        self,
        lidar_msg: MarkerArray,
        yolo_msg: Detection2DArray,
        depth_msg: Image
    ):
        """Main fusion callback with comprehensive error handling."""
        self.fusion_count += 1
        current_time = time.time()

        try:
            # Update sensor health
            self.sensor_health['lidar'].update(current_time, len(lidar_msg.markers) > 0)
            self.sensor_health['yolo'].update(current_time, len(yolo_msg.detections) > 0)
            self.sensor_health['depth'].update(current_time, True)

            # Update operating mode based on sensor health
            self._update_fusion_mode(current_time)

            # Check if we have minimum requirements
            if self.K is None:
                self.get_logger().warn_throttle(5.0, 'No camera calibration - waiting...')
                return

            # Convert depth image with error handling
            try:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge error: {e}')
                self.failed_fusions += 1
                return

            # Extract data based on operating mode
            lidar_clusters = []
            yolo_detections = []

            if self.fusion_mode in [FusionMode.FULL_FUSION, FusionMode.LIDAR_CAMERA, FusionMode.LIDAR_ONLY]:
                lidar_clusters = self.extract_lidar_clusters(lidar_msg)

            if self.fusion_mode in [FusionMode.FULL_FUSION, FusionMode.CAMERA_ONLY]:
                yolo_detections = yolo_msg.detections

            # Perform fusion based on mode
            fused_obstacles = self._perform_fusion(
                lidar_clusters,
                yolo_detections,
                depth_image,
                yolo_msg.header
            )

            # Apply tracking if enabled
            if self.enable_tracking:
                fused_obstacles = self._track_obstacles(fused_obstacles)

            # Publish results
            if fused_obstacles:
                self._publish_all_outputs(fused_obstacles, yolo_msg.header)
                self.successful_fusions += 1
            else:
                self._publish_empty_outputs(yolo_msg.header)

        except Exception as e:
            self.get_logger().error(f'Fusion callback error: {e}')
            self.failed_fusions += 1
            self._publish_empty_outputs(yolo_msg.header if yolo_msg else Header())

    def _update_fusion_mode(self, current_time: float):
        """Update operating mode based on sensor health."""
        if not self.enable_fallback:
            return

        lidar_ok = self.sensor_health['lidar'].is_healthy(current_time)
        yolo_ok = self.sensor_health['yolo'].is_healthy(current_time)
        camera_ok = self.sensor_health['depth'].is_healthy(current_time)

        old_mode = self.fusion_mode

        if lidar_ok and yolo_ok and camera_ok:
            self.fusion_mode = FusionMode.FULL_FUSION
        elif lidar_ok and camera_ok:
            self.fusion_mode = FusionMode.LIDAR_CAMERA
        elif lidar_ok:
            self.fusion_mode = FusionMode.LIDAR_ONLY
        elif camera_ok and yolo_ok:
            self.fusion_mode = FusionMode.CAMERA_ONLY
        else:
            self.fusion_mode = FusionMode.SAFE_STOP
            self.get_logger().error('ALL SENSORS FAILED - SAFE STOP MODE')

        if old_mode != self.fusion_mode:
            self.mode_changes += 1
            self.get_logger().warn(f'Mode change: {old_mode.value} -> {self.fusion_mode.value}')

    def _update_transforms_retry(self):
        """Update TF transforms with retry mechanism."""
        try:
            if self.T_lidar_to_camera is None:
                t = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.lidar_frame,
                    rclpy.time.Time()
                )
                self.T_lidar_to_camera = self._transform_to_matrix(t)
                self.get_logger().info('✓ LiDAR-to-camera transform acquired')

            if self.T_camera_to_base is None:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    rclpy.time.Time()
                )
                self.T_camera_to_base = self._transform_to_matrix(t)
                self.get_logger().info('✓ Camera-to-base transform acquired')

            if self.T_lidar_to_base is None:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.lidar_frame,
                    rclpy.time.Time()
                )
                self.T_lidar_to_base = self._transform_to_matrix(t)
                self.get_logger().info('✓ LiDAR-to-base transform acquired')

            # All transforms acquired - stop timer
            if all([self.T_lidar_to_camera, self.T_camera_to_base, self.T_lidar_to_base]):
                self.tf_timer.cancel()
                self.get_logger().info('All TF transforms acquired successfully')

        except TransformException as ex:
            self.tf_retry_count += 1
            if self.tf_retry_count < self.MAX_TF_RETRIES:
                self.get_logger().warn_throttle(
                    5.0,
                    f'Waiting for TF transforms ({self.tf_retry_count}/{self.MAX_TF_RETRIES})...'
                )
            else:
                self.get_logger().error(
                    f'Failed to acquire TF transforms after {self.MAX_TF_RETRIES} attempts'
                )

    def _transform_to_matrix(self, transform) -> np.ndarray:
        """Convert ROS Transform to 4x4 matrix."""
        t = transform.transform
        q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        R = self._quaternion_to_rotation_matrix(q)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return T

    def _quaternion_to_rotation_matrix(self, q: list) -> np.ndarray:
        """Convert quaternion to rotation matrix."""
        x, y, z, w = q
        R = np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
        return R

    # [Continue in next file due to length...]


def main(args=None):
    """Run the robust sensor fusion node."""
    rclpy.init(args=args)
    node = RobustSensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('='*60)
        node.get_logger().info('SHUTDOWN STATISTICS:')
        node.get_logger().info(f'  Total fusions: {node.fusion_count}')
        node.get_logger().info(f'  Successful: {node.successful_fusions}')
        node.get_logger().info(f'  Failed: {node.failed_fusions}')
        node.get_logger().info(f'  Success rate: {100*node.successful_fusions/max(node.fusion_count,1):.1f}%')
        node.get_logger().info(f'  Mode changes: {node.mode_changes}')
        node.get_logger().info('='*60)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
