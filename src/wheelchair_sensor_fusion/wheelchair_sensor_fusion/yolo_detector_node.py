#!/usr/bin/env python3
"""YOLOv11 Object Detection Node for Wheelchair Sensor Fusion.

This node performs real-time object detection using YOLOv11 on camera images
and publishes detection results for sensor fusion.

Author: Siddharth Tiwari
Email: s24035@students.iitmandi.ac.in
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import time

# Try to import ultralytics YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class YOLODetectorNode(Node):
    """YOLOv11-based object detection node."""

    def __init__(self):
        """Initialize the YOLO detector node."""
        super().__init__('yolo_detector_node')

        # Check if YOLO is available
        if not YOLO_AVAILABLE:
            self.get_logger().error(
                'Ultralytics YOLO not found! Please install: pip install ultralytics'
            )
            raise ImportError('ultralytics package not found')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter('model_path', 'yolov11n.pt')  # nano model by default
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cuda')  # 'cuda' or 'cpu'
        self.declare_parameter('img_size', 640)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('target_classes', [])  # Empty = all classes

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.debug_topic = self.get_parameter('debug_image_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.device = self.get_parameter('device').value
        self.img_size = self.get_parameter('img_size').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.target_classes = self.get_parameter('target_classes').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera info
        self.camera_info = None

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
            self.get_logger().info(
                f'YOLO model loaded successfully on {self.device}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        # Publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            self.detections_topic,
            10
        )

        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image,
                self.debug_topic,
                10
            )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.total_inference_time = 0.0

        self.get_logger().info(
            f'YOLO Detector initialized - Listening to {self.image_topic}'
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration info.

        Args:
            msg: CameraInfo message
        """
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received')

    def image_callback(self, msg: Image):
        """Process incoming images with YOLO.

        Args:
            msg: Image message from camera
        """
        self.frame_count += 1

        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Run YOLO inference
        start_time = time.time()
        detections = self.detect_objects(cv_image)
        inference_time = time.time() - start_time
        self.total_inference_time += inference_time

        # Publish detections
        detection_array = self.create_detection_array(
            detections,
            msg.header
        )
        self.detections_pub.publish(detection_array)

        self.detection_count += len(detections)

        # Publish debug image
        if self.publish_debug and len(detections) > 0:
            debug_image = self.draw_detections(cv_image, detections)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

        # Log statistics
        if self.frame_count % 100 == 0:
            avg_time = self.total_inference_time / self.frame_count
            fps = 1.0 / avg_time if avg_time > 0 else 0
            self.get_logger().info(
                f'Processed {self.frame_count} frames, '
                f'{self.detection_count} detections, '
                f'avg inference: {avg_time*1000:.1f}ms ({fps:.1f} FPS)'
            )

    def detect_objects(self, image: np.ndarray) -> List[dict]:
        """Run YOLO detection on image.

        Args:
            image: OpenCV image (BGR format)

        Returns:
            List of detections, each containing:
                - bbox: [x1, y1, x2, y2]
                - confidence: float
                - class_id: int
                - class_name: str
        """
        # Run inference
        results = self.model.predict(
            image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.img_size,
            verbose=False,
            classes=self.target_classes if self.target_classes else None
        )

        detections = []

        # Parse results
        if len(results) > 0:
            result = results[0]
            boxes = result.boxes

            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Get confidence and class
                confidence = float(box.conf[0].cpu().numpy())
                class_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[class_id]

                detection = {
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'confidence': confidence,
                    'class_id': class_id,
                    'class_name': class_name
                }

                detections.append(detection)

        return detections

    def create_detection_array(
        self,
        detections: List[dict],
        header: Header
    ) -> Detection2DArray:
        """Convert detections to ROS Detection2DArray message.

        Args:
            detections: List of detection dictionaries
            header: ROS message header

        Returns:
            Detection2DArray message
        """
        detection_array = Detection2DArray()
        detection_array.header = header

        for det in detections:
            detection_msg = Detection2D()
            detection_msg.header = header

            # Bounding box
            x1, y1, x2, y2 = det['bbox']
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            size_x = x2 - x1
            size_y = y2 - y1

            detection_msg.bbox.center.position.x = center_x
            detection_msg.bbox.center.position.y = center_y
            detection_msg.bbox.center.theta = 0.0
            detection_msg.bbox.size_x = size_x
            detection_msg.bbox.size_y = size_y

            # Object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = det['confidence']

            detection_msg.results.append(hypothesis)

            # Store class name in source_img (workaround for missing field)
            detection_msg.id = det['class_name']

            detection_array.detections.append(detection_msg)

        return detection_array

    def draw_detections(
        self,
        image: np.ndarray,
        detections: List[dict]
    ) -> np.ndarray:
        """Draw bounding boxes and labels on image.

        Args:
            image: OpenCV image
            detections: List of detections

        Returns:
            Image with drawn detections
        """
        debug_image = image.copy()

        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det['bbox']]
            confidence = det['confidence']
            class_name = det['class_name']

            # Draw bounding box
            color = self.get_class_color(det['class_id'])
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)

            # Draw label
            label = f'{class_name}: {confidence:.2f}'
            label_size, _ = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                1
            )
            cv2.rectangle(
                debug_image,
                (x1, y1 - label_size[1] - 5),
                (x1 + label_size[0], y1),
                color,
                -1
            )
            cv2.putText(
                debug_image,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

        return debug_image

    def get_class_color(self, class_id: int) -> Tuple[int, int, int]:
        """Get a unique color for each class.

        Args:
            class_id: Class identifier

        Returns:
            BGR color tuple
        """
        colors = [
            (255, 0, 0),    # Blue
            (0, 255, 0),    # Green
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
        ]
        return colors[class_id % len(colors)]


def main(args=None):
    """Run the YOLO detector node."""
    rclpy.init(args=args)
    node = YOLODetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down - Processed {node.frame_count} frames, '
            f'{node.detection_count} detections'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
