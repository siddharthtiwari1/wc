#!/usr/bin/env python3
"""
YOLO-World + SAM2 Detection Pipeline for CapNav-Real

Combines:
- YOLO-World: Open-vocabulary detection
- SAM2: Precise segmentation
- Real-time processing for wheelchair navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import torch
import cv2

# Detection message (create custom msg later)
from geometry_msgs.msg import Point


class YOLOSAMDetector(Node):
    """Unified detection + segmentation node."""

    def __init__(self):
        super().__init__('yolo_sam_detector')

        # Parameters
        self.declare_parameter('yolo_model', 'yolov8x-worldv2.pt')
        self.declare_parameter('sam_model', 'sam2_hiera_large.pt')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')

        self.device = self.get_parameter('device').value
        self.conf_thresh = self.get_parameter('conf_threshold').value

        self.get_logger().info(f'Initializing detection pipeline on {self.device}...')

        # Load models
        try:
            from ultralytics import YOLO
            self.yolo = YOLO(self.get_parameter('yolo_model').value)
            self.yolo.set_classes(self.get_indoor_vocabulary())
            self.get_logger().info('YOLO-World loaded')
        except Exception as e:
            self.get_logger().error(f'YOLO init failed: {e}')

        try:
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            self.sam = SAM2ImagePredictor.from_pretrained(
                self.get_parameter('sam_model').value
            )
            self.get_logger().info('SAM2 loaded')
        except Exception as e:
            self.get_logger().warn(f'SAM2 init failed (optional): {e}')
            self.sam = None

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.detect_callback,
            10
        )

        # Publishers (will create custom Detection message)
        # For now, publish annotated image
        self.vis_pub = self.create_publisher(Image, '/capnav/detections_vis', 10)

        self.get_logger().info('Detector ready')

    def get_indoor_vocabulary(self):
        """Return indoor object vocabulary."""
        return [
            # Furniture
            'chair', 'table', 'desk', 'bed', 'sofa', 'couch', 'bench',
            'cabinet', 'shelf', 'bookshelf', 'wardrobe', 'drawer',

            # Appliances
            'refrigerator', 'microwave', 'oven', 'dishwasher', 'washing machine',
            'television', 'monitor', 'laptop', 'computer',

            # Bathroom
            'toilet', 'sink', 'bathtub', 'shower', 'mirror',

            # Kitchen
            'stove', 'countertop', 'cutting board', 'toaster',

            # Decor
            'plant', 'vase', 'picture', 'painting', 'clock', 'lamp',
            'pillow', 'cushion', 'blanket', 'curtain', 'rug', 'carpet',

            # Objects
            'bottle', 'cup', 'mug', 'bowl', 'plate', 'book', 'towel',
            'trash can', 'basket', 'box', 'bag',

            # Doors & Windows
            'door', 'window', 'wall', 'floor'
        ]

    def detect_callback(self, msg):
        """Run YOLO detection + SAM segmentation."""
        try:
            # Convert to CV2
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLO detection
            results = self.yolo.predict(
                cv_image,
                conf=self.conf_thresh,
                device=self.device,
                verbose=False
            )

            if len(results) == 0 or len(results[0].boxes) == 0:
                return

            detections = results[0].boxes

            # Optional: SAM refinement
            if self.sam is not None:
                # Set image for SAM
                self.sam.set_image(cv_image)

            # Process each detection
            annotated = cv_image.copy()
            for box in detections:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.yolo.names[cls]

                # Draw bbox
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f'{label} {conf:.2f}',
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                # Optional SAM segmentation
                if self.sam is not None:
                    masks, scores, _ = self.sam.predict(
                        box=np.array([x1, y1, x2, y2]),
                        multimask_output=False
                    )
                    if len(masks) > 0:
                        mask = masks[0]
                        # Overlay mask
                        mask_overlay = annotated.copy()
                        mask_overlay[mask > 0] = [0, 0, 255]
                        annotated = cv2.addWeighted(annotated, 0.7, mask_overlay, 0.3, 0)

            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)

            # TODO: Publish custom Detection message with:
            # - bbox, mask, label, confidence
            # - Will be consumed by instance_clusterer.py

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOSAMDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
