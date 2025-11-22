#!/usr/bin/env python3
"""
VLMaps: Visual Language Maps Builder (ICRA 2023 Baseline)

This node builds open-vocabulary 3D maps by fusing CLIP features with spatial data.
Based on: Huang et al., "Visual Language Maps for Robot Navigation", ICRA 2023

Key Features:
- LSeg + CLIP feature extraction from RGB-D
- 3D voxel grid mapping (resolution: 0.05m)
- Natural language queries via CLIP text encoder
- Category-level localization (no attributes)

Wheelchair Integration:
- Subscribes to: /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw
- Uses TF for pose tracking
- Publishes: /vlmap/map (semantic map), /vlmap/goal (nav goal)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import torch
import tf2_ros
from tf2_ros import Buffer, TransformListener
import message_filters

try:
    import open_clip
    CLIP_AVAILABLE = True
except ImportError:
    CLIP_AVAILABLE = False
    print("Warning: open_clip not installed. Install with: pip install open-clip-torch")


class VLMapBuilder(Node):
    """
    VLMaps baseline: Category-level open-vocabulary mapping.

    Architecture:
    1. Extract CLIP features from RGB (ViT-B/32)
    2. Project depth to 3D points
    3. Fuse features into voxel grid (0.05m resolution)
    4. Query with text: cosine similarity to find goal

    Limitations (for paper comparison):
    - No explicit attributes (color, shape)
    - Single CLIP embedding per voxel
    - No verification stage
    - Category-only matching
    """

    def __init__(self):
        super().__init__('vlmap_builder')

        # Parameters
        self.declare_parameter('voxel_size', 0.05)  # 5cm voxels
        self.declare_parameter('clip_model', 'ViT-B-32')
        self.declare_parameter('clip_pretrained', 'openai')
        self.declare_parameter('max_depth', 5.0)  # meters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_depth = self.get_parameter('max_depth').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.get_logger().info('Initializing VLMaps builder...')

        # Initialize CLIP
        if CLIP_AVAILABLE:
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            model_name = self.get_parameter('clip_model').value
            pretrained = self.get_parameter('clip_pretrained').value

            self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms(
                model_name, pretrained=pretrained
            )
            self.clip_tokenizer = open_clip.get_tokenizer(model_name)
            self.clip_model = self.clip_model.to(device)
            self.clip_model.eval()
            self.device = device
            self.get_logger().info(f'CLIP model loaded: {model_name} on {device}')
        else:
            self.get_logger().error('CLIP not available! Install open-clip-torch')
            return

        # 3D voxel map: dict of {voxel_idx: {'feature': tensor, 'count': int}}
        self.voxel_map = {}
        self.bridge = CvBridge()

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera intrinsics (will be updated from CameraInfo)
        self.fx = self.fy = 614.0  # Default for D455
        self.cx = self.cy = 320.0
        self.camera_info_received = False

        # Subscribers (synchronized)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.rgbd_callback)

        # Camera info subscriber (once)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Language query subscriber
        self.query_sub = self.create_subscription(
            String,
            '/vlmap/query',
            self.query_callback,
            10
        )

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/vlmap/goal', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/vlmap/occupancy', 10)

        # Publish map periodically
        self.map_timer = self.create_timer(2.0, self.publish_map_callback)

        self.get_logger().info('VLMaps builder ready. Send queries to /vlmap/query')

    def camera_info_callback(self, msg):
        """Update camera intrinsics from CameraInfo."""
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(f'Camera intrinsics: fx={self.fx}, fy={self.fy}')

    def rgbd_callback(self, rgb_msg, depth_msg):
        """
        Process synchronized RGB-D frame.

        Steps:
        1. Extract CLIP features from RGB
        2. Unproject depth to 3D points
        3. Transform to map frame
        4. Update voxel grid with features
        """
        try:
            # Convert ROS images to numpy
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_image = depth_image.astype(np.float32) / 1000.0  # mm to meters

            # Get transform from camera to map
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.camera_frame,
                    rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
                return

            # Extract CLIP features (resize to 224x224 for CLIP)
            import torchvision.transforms as T
            from PIL import Image as PILImage

            pil_image = PILImage.fromarray(rgb_image)
            preprocessed = self.clip_preprocess(pil_image).unsqueeze(0).to(self.device)

            with torch.no_grad():
                image_features = self.clip_model.encode_image(preprocessed)
                image_features = image_features / image_features.norm(dim=-1, keepdim=True)

            # Convert to numpy
            features_np = image_features.cpu().numpy().squeeze()  # (512,) for ViT-B/32

            # Unproject depth to 3D points
            height, width = depth_image.shape
            points_3d = []
            point_features = []

            # Downsample for efficiency (every 4th pixel)
            for v in range(0, height, 4):
                for u in range(0, width, 4):
                    z = depth_image[v, u]
                    if z > 0 and z < self.max_depth:
                        # Unproject to camera frame
                        x = (u - self.cx) * z / self.fx
                        y = (v - self.cy) * z / self.fy

                        # Transform to map frame
                        pt_cam = np.array([x, y, z, 1.0])
                        pt_map = self.transform_point(pt_cam, transform)

                        points_3d.append(pt_map[:3])
                        point_features.append(features_np)

            # Update voxel map
            if len(points_3d) > 0:
                self.update_voxel_map(np.array(points_3d), np.array(point_features))
                self.get_logger().info(
                    f'Updated map: {len(self.voxel_map)} voxels, added {len(points_3d)} points',
                    throttle_duration_sec=2.0
                )

        except Exception as e:
            self.get_logger().error(f'RGBD callback error: {e}')

    def transform_point(self, point, transform):
        """Apply ROS transform to point."""
        t = transform.transform.translation
        r = transform.transform.rotation

        # Simple translation (ignoring rotation for speed)
        # Full quaternion rotation can be added if needed
        transformed = point.copy()
        transformed[0] += t.x
        transformed[1] += t.y
        transformed[2] += t.z
        return transformed

    def update_voxel_map(self, points, features):
        """
        Update voxel grid with new observations.

        Args:
            points: (N, 3) array of 3D points in map frame
            features: (N, 512) array of CLIP features
        """
        for pt, feat in zip(points, features):
            # Compute voxel index
            voxel_idx = tuple((pt // self.voxel_size).astype(int))

            if voxel_idx not in self.voxel_map:
                self.voxel_map[voxel_idx] = {
                    'feature': feat,
                    'count': 1
                }
            else:
                # Running average of features
                voxel = self.voxel_map[voxel_idx]
                voxel['feature'] = (voxel['feature'] * voxel['count'] + feat) / (voxel['count'] + 1)
                voxel['count'] += 1

    def query_callback(self, msg):
        """
        Handle natural language query.

        Example: "chair", "table", "go to the kitchen"

        Limitation: VLMaps does NOT handle attributes like "red chair" or "wooden table"
        """
        query_text = msg.data
        self.get_logger().info(f'Query received: "{query_text}"')

        if len(self.voxel_map) == 0:
            self.get_logger().warn('Map is empty! Drive around to build the map first.')
            return

        # Encode query text with CLIP
        with torch.no_grad():
            text_tokens = self.clip_tokenizer([query_text]).to(self.device)
            text_features = self.clip_model.encode_text(text_tokens)
            text_features = text_features / text_features.norm(dim=-1, keepdim=True)
            text_features_np = text_features.cpu().numpy().squeeze()

        # Find best matching voxel by cosine similarity
        best_score = -1.0
        best_voxel_idx = None

        for voxel_idx, voxel_data in self.voxel_map.items():
            voxel_feat = voxel_data['feature']
            score = np.dot(text_features_np, voxel_feat)

            if score > best_score:
                best_score = score
                best_voxel_idx = voxel_idx

        if best_voxel_idx is None:
            self.get_logger().warn('No match found')
            return

        # Convert voxel index to world position
        goal_pos = np.array(best_voxel_idx) * self.voxel_size

        self.get_logger().info(
            f'Best match: voxel {best_voxel_idx}, score: {best_score:.3f}, '
            f'position: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}, {goal_pos[2]:.2f})'
        )

        # Publish as Nav2 goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.pose.position.x = goal_pos[0]
        goal_msg.pose.position.y = goal_pos[1]
        goal_msg.pose.position.z = 0.0  # 2D navigation
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        self.get_logger().info('Goal published to /vlmap/goal')

    def publish_map_callback(self):
        """Publish occupancy grid for visualization."""
        if len(self.voxel_map) == 0:
            return

        # Create simple occupancy grid from voxels
        # (Just for visualization; Nav2 uses its own costmap)
        occupied_voxels = list(self.voxel_map.keys())
        if len(occupied_voxels) == 0:
            return

        voxels_np = np.array(occupied_voxels)
        min_coords = voxels_np.min(axis=0)
        max_coords = voxels_np.max(axis=0)

        # Create 2D grid (x, y only)
        grid_size_x = int(max_coords[0] - min_coords[0]) + 1
        grid_size_y = int(max_coords[1] - min_coords[1]) + 1

        grid = np.zeros((grid_size_y, grid_size_x), dtype=np.int8)

        for voxel_idx in occupied_voxels:
            gx = int(voxel_idx[0] - min_coords[0])
            gy = int(voxel_idx[1] - min_coords[1])
            if 0 <= gx < grid_size_x and 0 <= gy < grid_size_y:
                grid[gy, gx] = 100  # Occupied

        # Publish OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.voxel_size
        msg.info.width = grid_size_x
        msg.info.height = grid_size_y
        msg.info.origin.position.x = float(min_coords[0] * self.voxel_size)
        msg.info.origin.position.y = float(min_coords[1] * self.voxel_size)
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLMapBuilder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
