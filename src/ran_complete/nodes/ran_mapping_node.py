#!/usr/bin/env python3
"""
RAN Mapping Node - COMPLETE IMPLEMENTATION
Novel Adaptive Clustering Algorithm for Attribute-Aware Navigation

Based on:
- CapNav (ICLR 2026): Multi-view captioning + instance clustering
- O3D-SIM (2024): Instance-level 3D semantic maps
- VLMaps (ICRA 2023): Open-vocabulary feature fusion

NOVEL CONTRIBUTIONS:
1. Adaptive threshold calibration (τ_sem based on scene complexity)
2. Confidence-weighted multi-view fusion
3. Dynamic instance tracking with temporal consistency
4. Per-attribute confidence propagation

Author: Siddharth Tiwari
Target: RSS/ICRA 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import TransformStamped, Point, Pose
from std_msgs.msg import Header, String
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import torch
import cv2
from scipy.spatial import cKDTree
import json
import pickle
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import tf2_ros
from cv_bridge import CvBridge


class SemanticInstance:
    """Represents a persistent object instance with attributes and confidence."""

    def __init__(self, instance_id: int, label: str, initial_feature: np.ndarray):
        self.id = instance_id
        self.label = label
        self.feature = initial_feature  # Multi-modal embedding (DINOv2 + CLIP)
        self.points_3d = []  # List of 3D points (world frame)
        self.observations = []  # List of (frame_id, bbox, quality, timestamp)
        self.attributes = {
            'color': {'value': None, 'confidence': 0.0},
            'shape': {'value': None, 'confidence': 0.0},
            'material': {'value': None, 'confidence': 0.0},
            'location': {'value': None, 'confidence': 0.0}
        }
        self.centroid = None
        self.bbox_3d = None  # Axis-aligned bounding box
        self.confidence = 1.0  # Overall instance confidence
        self.last_seen = None
        self.caption = None
        self.best_view_frame = None  # Frame ID with best quality score

    def update_feature(self, new_feature: np.ndarray, quality: float):
        """Confidence-weighted feature update (NOVEL)."""
        total_quality = sum([obs[2] for obs in self.observations]) + quality
        self.feature = (self.feature * (total_quality - quality) + new_feature * quality) / total_quality

    def add_observation(self, frame_id: int, bbox: List[int], points: np.ndarray, quality: float, timestamp):
        """Add new observation with quality score."""
        self.observations.append((frame_id, bbox, quality, timestamp))
        self.points_3d.extend(points.tolist())
        self.last_seen = timestamp

        # Update best view
        if self.best_view_frame is None or quality > self.get_best_quality():
            self.best_view_frame = frame_id

    def get_best_quality(self) -> float:
        """Get quality score of best observation."""
        if not self.observations:
            return 0.0
        return max([obs[2] for obs in self.observations])

    def compute_centroid(self):
        """Compute 3D centroid from all points."""
        if self.points_3d:
            self.centroid = np.mean(self.points_3d, axis=0)

    def compute_bbox_3d(self):
        """Compute axis-aligned 3D bounding box."""
        if self.points_3d:
            points = np.array(self.points_3d)
            self.bbox_3d = {
                'min': points.min(axis=0),
                'max': points.max(axis=0)
            }

    def to_dict(self) -> Dict:
        """Serialize instance to JSON-compatible dict."""
        return {
            'id': self.id,
            'label': self.label,
            'feature': self.feature.tolist() if isinstance(self.feature, np.ndarray) else self.feature,
            'centroid': self.centroid.tolist() if self.centroid is not None else None,
            'bbox_3d': {
                'min': self.bbox_3d['min'].tolist(),
                'max': self.bbox_3d['max'].tolist()
            } if self.bbox_3d else None,
            'attributes': self.attributes,
            'confidence': float(self.confidence),
            'num_observations': len(self.observations),
            'caption': self.caption,
            'last_seen': str(self.last_seen)
        }


class RANMappingNode(Node):
    """
    Complete 3D Semantic Mapping with Adaptive Clustering.

    Pipeline:
    1. Receive detections from perception node
    2. Project depth to 3D points
    3. Transform to map frame
    4. Adaptive instance clustering (NOVEL)
    5. Multi-view attribute extraction
    6. Build persistent semantic map
    """

    def __init__(self):
        super().__init__('ran_mapping_node')

        self.get_logger().info('='*70)
        self.get_logger().info('RAN MAPPING NODE - Adaptive Clustering & Attribute Extraction')
        self.get_logger().info('='*70)

        # Parameters
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        # NOVEL: Adaptive clustering parameters
        self.declare_parameter('adaptive_tau_sem', True)
        self.declare_parameter('tau_sem_base', 0.65)
        self.declare_parameter('tau_sem_scaling', 0.1)
        self.declare_parameter('tau_vol', 0.15)
        self.declare_parameter('tau_iou', 0.20)
        self.declare_parameter('tau_cen', 0.15)
        self.declare_parameter('min_points_per_instance', 50)

        # Multi-view fusion
        self.declare_parameter('use_quality_weighting', True)
        self.declare_parameter('min_quality_threshold', 0.3)

        # Map saving
        self.declare_parameter('map_save_dir', str(Path.home() / '.ros' / 'ran_maps'))
        self.declare_parameter('auto_save_interval', 60.0)  # seconds

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_depth = self.get_parameter('max_depth').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.adaptive_tau = self.get_parameter('adaptive_tau_sem').value
        self.tau_sem_base = self.get_parameter('tau_sem_base').value
        self.tau_sem_scaling = self.get_parameter('tau_sem_scaling').value
        self.tau_vol = self.get_parameter('tau_vol').value
        self.tau_iou = self.get_parameter('tau_iou').value
        self.tau_cen = self.get_parameter('tau_cen').value

        # Semantic map storage
        self.instances: Dict[int, SemanticInstance] = {}
        self.next_instance_id = 0
        self.frame_count = 0

        # Detection buffer (from perception node)
        self.detection_buffer = []

        # Camera intrinsics
        self.fx = self.fy = 614.0
        self.cx = self.cy = 320.0
        self.camera_info_received = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        # Map save directory
        self.map_save_dir = Path(self.get_parameter('map_save_dir').value)
        self.map_save_dir.mkdir(parents=True, exist_ok=True)

        # Publishers
        self.map_pub = self.create_publisher(MarkerArray, '/ran/semantic_map_markers', 10)
        self.map_status_pub = self.create_publisher(String, '/ran/map_status', 10)

        # Subscribers (will receive from perception node)
        # For now, subscribe to raw RGBD for testing
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_callback, 10
        )

        # Auto-save timer
        self.save_timer = self.create_timer(
            self.get_parameter('auto_save_interval').value,
            self.auto_save_map
        )

        # Visualization timer
        self.vis_timer = self.create_timer(2.0, self.publish_map_visualization)

        self.get_logger().info(f'Mapping node ready. Adaptive clustering: {self.adaptive_tau}')
        self.get_logger().info(f'Map will be saved to: {self.map_save_dir}')

    def camera_info_callback(self, msg):
        """Update camera intrinsics."""
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(f'Camera intrinsics updated: fx={self.fx:.1f}')

    def add_detection(self, detection: Dict):
        """
        Add detection from perception node and cluster into instances.

        Detection format:
        {
            'bbox': [x1, y1, x2, y2],
            'label': str,
            'confidence': float,
            'features': {'dinov2': np.array, 'clip': np.array},
            'quality': {'Q_total': float, ...},
            'attr_confidence': {'c_color': float, ...},
            'points_3d': np.array,  # (N, 3)
            'timestamp': rclpy.time.Time
        }
        """
        self.detection_buffer.append(detection)
        self.frame_count += 1

        # Process detection
        self._cluster_detection(detection)

        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Processed {self.frame_count} frames, {len(self.instances)} instances')

    def _cluster_detection(self, detection: Dict):
        """
        NOVEL: Adaptive clustering algorithm.

        Steps:
        1. Compute adaptive τ_sem based on current map size
        2. Semantic gating: Find candidates with similar features/labels
        3. Geometric voting: Check spatial consistency (IoU, centroid, volume)
        4. Merge if ≥2 geometric cues agree
        5. Otherwise create new instance
        """
        # Extract detection data
        bbox = detection['bbox']
        label = detection['label']
        feature = self._combine_features(detection['features'])
        points_3d = detection.get('points_3d', np.array([]))
        quality = detection['quality']['Q_total']
        timestamp = detection['timestamp']

        if points_3d.size == 0 or quality < self.get_parameter('min_quality_threshold').value:
            return  # Skip low-quality detections

        # NOVEL: Compute adaptive threshold
        tau_sem = self._compute_adaptive_tau_sem()

        # Semantic gating
        candidates = []
        for inst_id, inst in self.instances.items():
            # Check label match OR feature similarity
            if inst.label == label:
                candidates.append((inst_id, inst, 1.0))  # Perfect match
            else:
                sim = self._cosine_similarity(feature, inst.feature)
                if sim > tau_sem:
                    candidates.append((inst_id, inst, sim))

        if not candidates:
            # Create new instance
            self._create_new_instance(label, feature, bbox, points_3d, quality, timestamp)
            return

        # Geometric voting
        best_match = None
        best_score = 0

        for inst_id, inst, sem_sim in candidates:
            votes = 0

            # Compute instance's current 3D representation
            if not inst.points_3d:
                continue

            inst_points = np.array(inst.points_3d)

            # Vote 1: Volumetric overlap (KD-tree based)
            vol_overlap = self._compute_volumetric_overlap(points_3d, inst_points)
            if vol_overlap > self.tau_vol:
                votes += 1

            # Vote 2: 3D IoU of bounding boxes
            if inst.bbox_3d is not None:
                iou_3d = self._compute_3d_iou(points_3d, inst_points)
                if iou_3d > self.tau_iou:
                    votes += 1

            # Vote 3: Centroid distance
            new_centroid = points_3d.mean(axis=0)
            if inst.centroid is not None:
                dist = np.linalg.norm(new_centroid - inst.centroid)
                if dist < self.tau_cen:
                    votes += 1

            # Require ≥2 votes to merge
            if votes >= 2:
                score = votes + sem_sim  # Combine geometric + semantic
                if score > best_score:
                    best_score = score
                    best_match = inst_id

        if best_match is not None:
            # Merge into existing instance
            self._merge_into_instance(best_match, bbox, points_3d, feature, quality, timestamp)
        else:
            # Create new instance
            self._create_new_instance(label, feature, bbox, points_3d, quality, timestamp)

    def _compute_adaptive_tau_sem(self) -> float:
        """
        NOVEL CONTRIBUTION: Adaptive semantic similarity threshold.

        Formula: τ_sem(N) = τ_base + τ_scaling * log(1 + N / 50)

        Intuition:
        - Few objects (N < 10): τ_sem ≈ 0.65 (relaxed, merge similar)
        - Many objects (N > 100): τ_sem ≈ 0.75 (strict, avoid false merges)
        """
        if not self.adaptive_tau:
            return self.tau_sem_base

        N = len(self.instances)
        tau = self.tau_sem_base + self.tau_sem_scaling * np.log(1 + N / 50.0)
        return float(tau)

    def _combine_features(self, features: Dict) -> np.ndarray:
        """Combine DINOv2 + CLIP features (weighted average)."""
        if 'dinov2' in features and 'clip' in features:
            dino = features['dinov2']
            clip = features['clip']
            # Normalize
            dino = dino / (np.linalg.norm(dino) + 1e-8)
            clip = clip / (np.linalg.norm(clip) + 1e-8)
            # Weighted combination (40% DINOv2, 60% CLIP)
            combined = 0.4 * dino + 0.6 * clip
            return combined / (np.linalg.norm(combined) + 1e-8)
        elif 'clip' in features:
            clip = features['clip']
            return clip / (np.linalg.norm(clip) + 1e-8)
        else:
            return np.zeros(512)  # Fallback

    def _cosine_similarity(self, a: np.ndarray, b: np.ndarray) -> float:
        """Cosine similarity between two vectors."""
        return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-8))

    def _compute_volumetric_overlap(self, points_a: np.ndarray, points_b: np.ndarray) -> float:
        """
        Compute volumetric overlap using KD-tree.

        Returns fraction of points_a within voxel_size of points_b.
        """
        if len(points_b) == 0:
            return 0.0

        tree = cKDTree(points_b)
        distances, _ = tree.query(points_a, k=1)
        overlap = np.sum(distances < self.voxel_size) / len(points_a)
        return float(overlap)

    def _compute_3d_iou(self, points_a: np.ndarray, points_b: np.ndarray) -> float:
        """Compute 3D IoU of axis-aligned bounding boxes."""
        if len(points_a) == 0 or len(points_b) == 0:
            return 0.0

        # Bounding boxes
        min_a, max_a = points_a.min(axis=0), points_a.max(axis=0)
        min_b, max_b = points_b.min(axis=0), points_b.max(axis=0)

        # Intersection
        inter_min = np.maximum(min_a, min_b)
        inter_max = np.minimum(max_a, max_b)
        inter_vol = np.prod(np.maximum(0, inter_max - inter_min))

        # Union
        vol_a = np.prod(max_a - min_a)
        vol_b = np.prod(max_b - min_b)
        union_vol = vol_a + vol_b - inter_vol

        return float(inter_vol / (union_vol + 1e-8))

    def _create_new_instance(self, label: str, feature: np.ndarray, bbox: List[int],
                            points: np.ndarray, quality: float, timestamp):
        """Create new semantic instance."""
        inst = SemanticInstance(self.next_instance_id, label, feature)
        inst.add_observation(self.frame_count, bbox, points, quality, timestamp)
        inst.compute_centroid()
        inst.compute_bbox_3d()

        self.instances[self.next_instance_id] = inst
        self.next_instance_id += 1

        self.get_logger().info(
            f'Created instance #{inst.id}: {label} (Q={quality:.2f})',
            throttle_duration_sec=1.0
        )

    def _merge_into_instance(self, inst_id: int, bbox: List[int], points: np.ndarray,
                            feature: np.ndarray, quality: float, timestamp):
        """Merge detection into existing instance with confidence weighting."""
        inst = self.instances[inst_id]

        # NOVEL: Confidence-weighted feature update
        inst.update_feature(feature, quality)
        inst.add_observation(self.frame_count, bbox, points, quality, timestamp)
        inst.compute_centroid()
        inst.compute_bbox_3d()

        # Update instance confidence based on consistency
        inst.confidence = self._compute_instance_confidence(inst)

    def _compute_instance_confidence(self, inst: SemanticInstance) -> float:
        """
        NOVEL: Compute instance confidence from multi-view consistency.

        Higher confidence if:
        - More observations
        - Higher average quality
        - Recent observations (temporal consistency)
        """
        if not inst.observations:
            return 0.0

        num_obs = len(inst.observations)
        avg_quality = np.mean([obs[2] for obs in inst.observations])

        # Temporal decay (prefer recent observations)
        current_time = self.get_clock().now()
        if inst.last_seen:
            time_since = (current_time - inst.last_seen).nanoseconds / 1e9
            temporal_factor = np.exp(-time_since / 10.0)  # Decay over 10 seconds
        else:
            temporal_factor = 1.0

        confidence = avg_quality * min(1.0, num_obs / 5.0) * temporal_factor
        return float(np.clip(confidence, 0.0, 1.0))

    def extract_attributes(self, inst_id: int, caption: str, attr_confidence: Dict):
        """
        Extract structured attributes from multi-view caption.

        Called after LLaVA captioning (will implement in separate node).
        """
        if inst_id not in self.instances:
            return

        inst = self.instances[inst_id]
        inst.caption = caption

        # Parse attributes from caption (simple keyword matching for now)
        # TODO: Use LLM for better parsing
        color_keywords = ['red', 'blue', 'green', 'yellow', 'white', 'black', 'brown', 'gray', 'grey']
        material_keywords = ['wood', 'wooden', 'metal', 'plastic', 'glass', 'ceramic', 'fabric', 'leather']

        caption_lower = caption.lower()

        # Extract color
        for color in color_keywords:
            if color in caption_lower:
                inst.attributes['color']['value'] = color
                inst.attributes['color']['confidence'] = attr_confidence.get('c_color', 0.5)
                break

        # Extract material
        for material in material_keywords:
            if material in caption_lower:
                inst.attributes['material']['value'] = material
                inst.attributes['material']['confidence'] = attr_confidence.get('c_material', 0.3)
                break

        # Shape and location require more sophisticated parsing
        inst.attributes['shape']['confidence'] = attr_confidence.get('c_shape', 0.5)

    def save_map(self, filename: Optional[str] = None) -> str:
        """Save semantic map to JSON file."""
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'ran_map_{timestamp}.json'

        filepath = self.map_save_dir / filename

        # Serialize map
        map_data = {
            'metadata': {
                'num_instances': len(self.instances),
                'num_frames': self.frame_count,
                'voxel_size': self.voxel_size,
                'created': datetime.now().isoformat()
            },
            'instances': {str(k): v.to_dict() for k, v in self.instances.items()}
        }

        with open(filepath, 'w') as f:
            json.dump(map_data, f, indent=2)

        self.get_logger().info(f'Map saved: {filepath}')
        return str(filepath)

    def load_map(self, filepath: str):
        """Load semantic map from JSON file."""
        with open(filepath, 'r') as f:
            map_data = json.load(f)

        # Clear current map
        self.instances.clear()

        # Reconstruct instances
        for inst_id_str, inst_dict in map_data['instances'].items():
            inst_id = int(inst_id_str)
            label = inst_dict['label']
            feature = np.array(inst_dict['feature'])

            inst = SemanticInstance(inst_id, label, feature)
            inst.centroid = np.array(inst_dict['centroid']) if inst_dict['centroid'] else None
            inst.confidence = inst_dict['confidence']
            inst.attributes = inst_dict['attributes']
            inst.caption = inst_dict.get('caption')

            self.instances[inst_id] = inst
            self.next_instance_id = max(self.next_instance_id, inst_id + 1)

        self.get_logger().info(f'Map loaded: {len(self.instances)} instances from {filepath}')

    def auto_save_map(self):
        """Auto-save map periodically."""
        if self.instances:
            self.save_map()

    def publish_map_visualization(self):
        """Publish map as RViz markers."""
        if not self.instances:
            return

        markers = MarkerArray()

        for inst_id, inst in self.instances.items():
            if inst.centroid is None:
                continue

            # Instance marker (sphere)
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'instances'
            marker.id = inst_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(inst.centroid[0])
            marker.pose.position.y = float(inst.centroid[1])
            marker.pose.position.z = float(inst.centroid[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = marker.scale.y = marker.scale.z = 0.2

            # Color by confidence (green = high, red = low)
            marker.color.r = 1.0 - inst.confidence
            marker.color.g = inst.confidence
            marker.color.b = 0.0
            marker.color.a = 0.8

            markers.markers.append(marker)

            # Label text
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'labels'
            text_marker.id = inst_id + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = float(inst.centroid[0])
            text_marker.pose.position.y = float(inst.centroid[1])
            text_marker.pose.position.z = float(inst.centroid[2]) + 0.3

            text_marker.text = f"{inst.label}\nconf:{inst.confidence:.2f}"
            text_marker.scale.z = 0.15
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            markers.markers.append(text_marker)

        self.map_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RANMappingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save map on shutdown
        if node.instances:
            node.save_map('ran_map_final.json')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
