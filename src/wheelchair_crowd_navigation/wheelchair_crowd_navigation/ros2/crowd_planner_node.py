#!/usr/bin/env python3
"""
ROS2 node for CrowdSurfer-based crowd navigation.

Subscribes to:
    - /scan (sensor_msgs/LaserScan)
    - /goal_pose (geometry_msgs/PoseStamped)
    - /odom (nav_msgs/Odometry)

Publishes:
    - /local_plan (nav_msgs/Path)
    - /cmd_vel (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import torch
import numpy as np
from pathlib import Path

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from wheelchair_crowd_navigation.models.vqvae import VQVAE
from wheelchair_crowd_navigation.models.pixelcnn import PixelCNN
from wheelchair_crowd_navigation.models.encoder import PerceptionEncoder
from wheelchair_crowd_navigation.planning.sampling_optimizer import SamplingOptimizer
from wheelchair_crowd_navigation.planning.trajectory_scorer import TrajectoryScorer


class CrowdPlannerNode(Node):
    """Main ROS2 node for crowd navigation."""

    def __init__(self):
        super().__init__('crowd_planner_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('planning_frequency', 10.0)
        self.declare_parameter('num_samples', 100)
        self.declare_parameter('num_elites', 10)
        self.declare_parameter('num_iterations', 5)

        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        planning_freq = self.get_parameter('planning_frequency').value
        num_samples = self.get_parameter('num_samples').value
        num_elites = self.get_parameter('num_elites').value
        num_iterations = self.get_parameter('num_iterations').value

        self.get_logger().info(f'Using device: {self.device}')

        # Initialize models
        self.get_logger().info('Initializing models...')
        self._init_models(model_path)

        # Initialize planner
        self.optimizer = SamplingOptimizer(
            vqvae_model=self.vqvae,
            pixelcnn_model=self.pixelcnn,
            num_samples=num_samples,
            num_elites=num_elites,
            num_iterations=num_iterations,
        )

        # Initialize trajectory scorer
        self.scorer = TrajectoryScorer()

        # State variables
        self.current_scan = None
        self.current_odom = None
        self.current_goal = None

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/local_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Planning timer
        self.planning_timer = self.create_timer(
            1.0 / planning_freq,
            self.planning_callback
        )

        self.get_logger().info('CrowdPlanner node initialized')

    def _init_models(self, model_path: str):
        """Initialize VQVAE and PixelCNN models."""
        # Initialize models
        self.perception_encoder = PerceptionEncoder(
            input_type='laserscan',
            input_dim=360,
            output_dim=512,
        ).to(self.device)

        self.vqvae = VQVAE(
            perception_dim=512,
            goal_dim=3,
            latent_dim=64,
            num_embeddings=512,
            horizon=30,
        ).to(self.device)

        self.pixelcnn = PixelCNN(
            num_embeddings=512,
            condition_dim=3,
        ).to(self.device)

        # Load weights if available
        if model_path and Path(model_path).exists():
            self.get_logger().info(f'Loading model from {model_path}')
            checkpoint = torch.load(model_path, map_location=self.device)

            if 'perception_encoder' in checkpoint:
                self.perception_encoder.load_state_dict(checkpoint['perception_encoder'])
            if 'vqvae' in checkpoint:
                self.vqvae.load_state_dict(checkpoint['vqvae'])
            if 'pixelcnn' in checkpoint:
                self.pixelcnn.load_state_dict(checkpoint['pixelcnn'])

            self.get_logger().info('Model loaded successfully')
        else:
            self.get_logger().warn('No model weights loaded - using random initialization')

        # Set to eval mode
        self.perception_encoder.eval()
        self.vqvae.eval()
        self.pixelcnn.eval()

    def scan_callback(self, msg: LaserScan):
        """Handle laser scan messages."""
        self.current_scan = np.array(msg.ranges)

    def odom_callback(self, msg: Odometry):
        """Handle odometry messages."""
        self.current_odom = msg

    def goal_callback(self, msg: PoseStamped):
        """Handle goal messages."""
        self.current_goal = msg
        self.get_logger().info(f'Received new goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def planning_callback(self):
        """Main planning loop."""
        # Check if we have all necessary data
        if self.current_scan is None:
            return

        if self.current_goal is None:
            return

        if self.current_odom is None:
            return

        try:
            # Get goal in robot frame
            goal_tensor = self._get_goal_tensor()

            # Get perception features
            perception_tensor = self._get_perception_tensor()

            # Define cost function
            def cost_fn(trajectories):
                return self.scorer.score_trajectories(
                    trajectories,
                    goal_tensor,
                    laser_scan=self.current_scan,
                )

            # Optimize trajectory
            with torch.no_grad():
                best_trajectory, cost = self.optimizer.optimize(
                    goal=goal_tensor.unsqueeze(0),
                    cost_function=cost_fn,
                    device=self.device,
                )

            # Publish trajectory
            self._publish_trajectory(best_trajectory)

            # Publish cmd_vel (first velocity command)
            self._publish_cmd_vel(best_trajectory)

        except Exception as e:
            self.get_logger().error(f'Planning error: {str(e)}')

    def _get_goal_tensor(self) -> torch.Tensor:
        """Convert goal to tensor in robot frame."""
        # For simplicity, assume goal is already in robot frame
        # In practice, use TF2 to transform

        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        # Extract yaw from quaternion (simplified)
        goal_theta = 0.0  # TODO: compute from quaternion

        goal = torch.tensor([goal_x, goal_y, goal_theta], dtype=torch.float32)

        return goal.to(self.device)

    def _get_perception_tensor(self) -> torch.Tensor:
        """Convert laser scan to perception features."""
        # Process laser scan
        scan = torch.from_numpy(self.current_scan).float().to(self.device)

        # Handle inf values
        scan = torch.where(torch.isinf(scan), torch.tensor(10.0), scan)

        # Encode
        with torch.no_grad():
            features = self.perception_encoder(scan.unsqueeze(0))

        return features.squeeze(0)

    def _publish_trajectory(self, trajectory: torch.Tensor):
        """Publish trajectory as Path message."""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'base_link'

        trajectory_np = trajectory.cpu().numpy()

        for i in range(len(trajectory_np)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(trajectory_np[i, 0])
            pose.pose.position.y = float(trajectory_np[i, 1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def _publish_cmd_vel(self, trajectory: torch.Tensor):
        """Publish velocity command."""
        # Simple controller: move towards first point
        trajectory_np = trajectory.cpu().numpy()

        if len(trajectory_np) < 2:
            return

        # Compute desired velocity
        dt = 0.1  # TODO: get from config
        target_point = trajectory_np[1]  # Next point

        # Compute linear and angular velocity
        linear_vel = np.linalg.norm(target_point) / dt
        angular_vel = np.arctan2(target_point[1], target_point[0]) / dt

        # Clip velocities
        linear_vel = np.clip(linear_vel, 0.0, 0.5)  # Max 0.5 m/s
        angular_vel = np.clip(angular_vel, -1.0, 1.0)  # Max 1.0 rad/s

        # Publish
        cmd_vel = Twist()
        cmd_vel.linear.x = float(linear_vel)
        cmd_vel.angular.z = float(angular_vel)

        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)

    node = CrowdPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
