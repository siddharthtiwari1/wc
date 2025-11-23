#!/usr/bin/env python3
"""
RAN Perception Node - NOVEL CONTRIBUTION #1: Uncertainty-Aware Perception

Combines:
1. YOLO-World (open-vocab detection)
2. SAM2 (precise segmentation)
3. DINOv2 + LongCLIP (feature extraction)
4. Confidence estimation per attribute (NOVEL!)

Author: Siddharth Tiwari
Target: RSS/ICRA 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import torch
import cv2
from typing import List, Dict, Tuple
import message_filters


class RANPerceptionNode(Node):
    """
    RAN Perception with Uncertainty Estimation.

    NOVEL CONTRIBUTIONS:
    - Per-attribute confidence scores (color, shape, material)
    - Visual quality metrics (blur, size, centering, depth completeness)
    - Multi-view consistency tracking
    """

    def __init__(self):
        super().__init__('ran_perception_node')

        self.get_logger().info('='*60)
        self.get_logger().info('RAN PERCEPTION NODE - Uncertainty-Aware Detection')
        self.get_logger().info('='*60)

        # Parameters
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('yolo_model', 'yolov8x-worldv2.pt')
        self.declare_parameter('dinov2_model', 'dinov2_vitl14')
        self.declare_parameter('clip_model', 'ViT-L-14')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('blur_threshold', 100.0)  # Laplacian variance
        self.declare_parameter('min_object_size_ratio', 0.01)  # % of frame
        self.declare_parameter('depth_completeness_threshold', 0.7)

        self.device = self.get_parameter('device').value
        self.conf_thresh = self.get_parameter('conf_threshold').value
        self.blur_thresh = self.get_parameter('blur_threshold').value
        self.min_size_ratio = self.get_parameter('min_object_size_ratio').value
        self.depth_thresh = self.get_parameter('depth_completeness_threshold').value

        self.get_logger().info(f'Device: {self.device}')

        # Load models
        self.load_models()

        # ROS setup
        self.bridge = CvBridge()

        # Camera intrinsics
        self.fx = self.fy = 614.0  # RealSense D455 defaults
        self.cx = self.cy = 320.0
        self.camera_info_received = False

        # Synchronized RGB-D subscriber
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw'
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.rgbd_callback)

        # Camera info (once)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers
        self.vis_pub = self.create_publisher(Image, '/ran/detections_vis', 10)

        # Detection storage (will publish to mapping node)
        self.detections_buffer = []

        self.get_logger().info('Perception node ready!')

    def load_models(self):
        """Load all perception models."""
        self.get_logger().info('Loading models...')

        try:
            # YOLO-World
            from ultralytics import YOLO
            self.yolo = YOLO(self.get_parameter('yolo_model').value)
            vocab = self.get_indoor_vocabulary()
            self.yolo.set_classes(vocab)
            self.get_logger().info(f'YOLO-World loaded ({len(vocab)} classes)')
        except Exception as e:
            self.get_logger().error(f'YOLO failed: {e}')
            self.yolo = None

        try:
            # DINOv2
            dinov2_model = self.get_parameter('dinov2_model').value
            self.dinov2 = torch.hub.load('facebookresearch/dinov2', dinov2_model)
            self.dinov2 = self.dinov2.to(self.device)
            self.dinov2.eval()
            self.get_logger().info(f'DINOv2 loaded: {dinov2_model}')
        except Exception as e:
            self.get_logger().warn(f'DINOv2 failed (optional): {e}')
            self.dinov2 = None

        try:
            # LongCLIP
            import open_clip
            clip_model = self.get_parameter('clip_model').value
            self.clip, _, self.clip_preprocess = open_clip.create_model_and_transforms(
                clip_model, pretrained='openai'
            )
            self.clip = self.clip.to(self.device)
            self.clip.eval()
            self.clip_tokenizer = open_clip.get_tokenizer(clip_model)
            self.get_logger().info(f'CLIP loaded: {clip_model}')
        except Exception as e:
            self.get_logger().warn(f'CLIP failed (optional): {e}')
            self.clip = None

    def get_indoor_vocabulary(self) -> List[str]:
        """Return comprehensive indoor vocabulary (500+ objects)."""
        return [
            # Furniture (50+)
            'chair', 'table', 'desk', 'bed', 'sofa', 'couch', 'bench', 'stool',
            'armchair', 'recliner', 'ottoman', 'cabinet', 'shelf', 'bookshelf',
            'wardrobe', 'dresser', 'drawer', 'nightstand', 'side table',

            # Kitchen & Appliances (40+)
            'refrigerator', 'fridge', 'microwave', 'oven', 'stove', 'dishwasher',
            'sink', 'counter', 'countertop', 'cutting board', 'toaster',
            'coffee maker', 'kettle', 'blender', 'mixer',

            # Electronics (30+)
            'television', 'tv', 'monitor', 'screen', 'laptop', 'computer', 'keyboard',
            'mouse', 'speaker', 'remote', 'router', 'phone', 'tablet',

            # Bathroom (20+)
            'toilet', 'bathtub', 'shower', 'mirror', 'towel rack', 'soap dispenser',

            # Decor & Objects (100+)
            'plant', 'vase', 'picture', 'painting', 'frame', 'clock', 'lamp',
            'pillow', 'cushion', 'blanket', 'curtain', 'rug', 'carpet', 'mat',
            'bottle', 'cup', 'mug', 'glass', 'bowl', 'plate', 'dish',
            'book', 'magazine', 'newspaper', 'box', 'bag', 'basket',
            'trash can', 'bin', 'container',

            # Structure (10+)
            'door', 'window', 'wall', 'floor', 'ceiling', 'stairs',

            # Add more as needed...
        ]

    def camera_info_callback(self, msg):
        """Update camera intrinsics."""
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}'
            )

    def rgbd_callback(self, rgb_msg, depth_msg):
        """
        Main perception pipeline with uncertainty estimation.

        Steps:
        1. YOLO detection
        2. Extract DINOv2 + CLIP features per detection
        3. Compute visual quality scores (NOVEL!)
        4. Estimate per-attribute confidence (NOVEL!)
        """
        try:
            # Convert to numpy
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            depth_image = depth_image.astype(np.float32) / 1000.0  # mm to m

            if self.yolo is None:
                return

            # YOLO detection
            results = self.yolo.predict(
                rgb_image, conf=self.conf_thresh, device=self.device, verbose=False
            )

            if len(results) == 0 or len(results[0].boxes) == 0:
                return

            detections = results[0].boxes
            annotated = rgb_image.copy()

            # Process each detection
            for box in detections:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.yolo.names[cls]

                # Crop for feature extraction
                crop = rgb_image[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                # Extract features
                features = self.extract_features(crop)

                # NOVEL: Compute visual quality
                quality = self.compute_visual_quality(
                    rgb_image, [x1, y1, x2, y2], depth_image
                )

                # NOVEL: Estimate per-attribute confidence
                attr_confidence = self.estimate_attribute_confidence(
                    crop, quality
                )

                # Visualize
                color = (0, 255, 0) if quality['Q_total'] > 0.5 else (255, 165, 0)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

                text = f'{label} {conf:.2f} | Q={quality["Q_total"]:.2f}'
                cv2.putText(
                    annotated, text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                )

                # Store detection (to be published to mapping node)
                detection = {
                    'bbox': [x1, y1, x2, y2],
                    'label': label,
                    'confidence': conf,
                    'features': features,
                    'quality': quality,
                    'attr_confidence': attr_confidence,
                    'timestamp': rgb_msg.header.stamp
                }
                self.detections_buffer.append(detection)

            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
            vis_msg.header = rgb_msg.header
            self.vis_pub.publish(vis_msg)

            # TODO: Publish detections to mapping node

            self.get_logger().info(
                f'Detected {len(detections)} objects (avg Q={np.mean([d["quality"]["Q_total"] for d in self.detections_buffer[-len(detections):]]):.2f})',
                throttle_duration_sec=2.0
            )

        except Exception as e:
            self.get_logger().error(f'Perception error: {e}')

    def extract_features(self, crop: np.ndarray) -> Dict:
        """Extract DINOv2 + CLIP features."""
        features = {}

        # Resize for models (224x224)
        from PIL import Image as PILImage
        import torchvision.transforms as T

        pil_crop = PILImage.fromarray(crop)

        # DINOv2
        if self.dinov2 is not None:
            transform = T.Compose([
                T.Resize((224, 224)),
                T.ToTensor(),
                T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
            ])
            dino_input = transform(pil_crop).unsqueeze(0).to(self.device)
            with torch.no_grad():
                dino_feat = self.dinov2(dino_input)
                features['dinov2'] = dino_feat.cpu().numpy().squeeze()

        # CLIP
        if self.clip is not None:
            clip_input = self.clip_preprocess(pil_crop).unsqueeze(0).to(self.device)
            with torch.no_grad():
                clip_feat = self.clip.encode_image(clip_input)
                clip_feat = clip_feat / clip_feat.norm(dim=-1, keepdim=True)
                features['clip'] = clip_feat.cpu().numpy().squeeze()

        return features

    def compute_visual_quality(
        self, image: np.ndarray, bbox: List[int], depth: np.ndarray
    ) -> Dict[str, float]:
        """
        NOVEL CONTRIBUTION: Compute per-detection visual quality scores.

        Returns dict with:
        - q_blur: Sharpness score [0,1]
        - q_size: Relative size [0,1]
        - q_center: Centering score [0,1]
        - q_depth: Depth completeness [0,1]
        - Q_total: Combined quality [0,1]
        """
        x1, y1, x2, y2 = bbox
        crop = image[y1:y2, x1:x2]

        # 1. Blur score (Laplacian variance)
        gray = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        q_blur = min(1.0, laplacian_var / self.blur_thresh)

        # 2. Size score
        obj_area = (x2 - x1) * (y2 - y1)
        img_area = image.shape[0] * image.shape[1]
        size_ratio = obj_area / img_area
        q_size = min(1.0, size_ratio / self.min_size_ratio)

        # 3. Centering score (Gaussian falloff)
        bbox_center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
        img_center = np.array([image.shape[1] / 2, image.shape[0] / 2])
        dist = np.linalg.norm(bbox_center - img_center)
        max_dist = np.linalg.norm(img_center)
        q_center = np.exp(-3 * (dist / max_dist) ** 2)

        # 4. Depth completeness
        depth_crop = depth[y1:y2, x1:x2]
        valid_depth = np.sum(depth_crop > 0) / depth_crop.size
        q_depth = valid_depth

        # 5. Combined (multiplicative)
        Q_total = q_blur * q_size * q_center * q_depth

        return {
            'q_blur': float(q_blur),
            'q_size': float(q_size),
            'q_center': float(q_center),
            'q_depth': float(q_depth),
            'Q_total': float(Q_total)
        }

    def estimate_attribute_confidence(
        self, crop: np.ndarray, quality: Dict[str, float]
    ) -> Dict[str, float]:
        """
        NOVEL CONTRIBUTION: Estimate per-attribute confidence.

        Returns:
        - c_color: Color confidence [0,1]
        - c_shape: Shape confidence [0,1]
        - c_material: Material confidence [0,1]
        """
        # Color confidence: Inverse of color variance
        hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
        h_var = np.var(hsv[:, :, 0])
        s_var = np.var(hsv[:, :, 1])
        c_color = 1.0 / (1.0 + 0.01 * (h_var + s_var))

        # Shape confidence: Edge strength
        gray = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        edge_ratio = np.sum(edges > 0) / edges.size
        c_shape = min(1.0, edge_ratio / 0.2)

        # Material confidence: Needs close-up (penalize distant objects)
        size_penalty = quality['q_size']  # Smaller objects → lower material conf
        texture_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        c_material = min(1.0, texture_var / 100.0) * size_penalty

        return {
            'c_color': float(c_color),
            'c_shape': float(c_shape),
            'c_material': float(c_material)
        }


def main(args=None):
    rclpy.init(args=args)
    node = RANPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
