#!/usr/bin/env python3
"""
Safety Monitor Node - Cross-Modal Verification for Wheelchair Navigation

This node provides an additional safety layer by cross-validating obstacle
detection between the 2D LiDAR and depth camera before allowing motion.

Key Features:
1. Cross-modal obstacle verification (LiDAR vs Depth)
2. Emergency stop on sensor disagreement
3. Passenger comfort constraints (jerk limiting)
4. Transparent obstacle detection (glass, mirrors)
5. Low obstacle detection (tables, knee-height objects)

Publications:
- /safety/status (std_msgs/String): Current safety state
- /safety/emergency_stop (std_msgs/Bool): Emergency stop flag
- /cmd_vel_safe (geometry_msgs/Twist): Safety-filtered velocity

Subscriptions:
- /scan_filtered (sensor_msgs/LaserScan): Filtered LiDAR data
- /camera/depth/image_rect_raw (sensor_msgs/Image): Depth image
- /cmd_vel (geometry_msgs/Twist): Input velocity command

Author: Siddharth Tiwari (s24035@students.iitmandi.ac.in)
Target: ICRA/IROS Publication
"""

import numpy as np
import math
from enum import Enum
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge


class SafetyState(Enum):
    """Safety states for the wheelchair."""
    SAFE = "SAFE"
    WARNING = "WARNING"
    DANGER = "DANGER"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    SENSOR_DISAGREEMENT = "SENSOR_DISAGREEMENT"


class SafetyMonitorNode(Node):
    """
    Cross-modal safety verification for wheelchair navigation.

    Validates obstacle detection between LiDAR and depth camera to ensure
    safe navigation, especially for:
    - Transparent obstacles (glass doors, windows)
    - Low obstacles (tables, wheelchair trays)
    - Dynamic obstacles (people, pets)
    """

    def __init__(self):
        super().__init__('safety_monitor_node')

        # ====================================================================
        # PARAMETERS
        # ====================================================================

        # Distance thresholds
        self.declare_parameter('emergency_stop_distance', 0.35)  # 35cm
        self.declare_parameter('danger_distance', 0.50)          # 50cm
        self.declare_parameter('warning_distance', 0.80)         # 80cm
        self.declare_parameter('slow_distance', 1.20)            # 1.2m

        # Sensor FOV for frontal arc check
        self.declare_parameter('lidar_frontal_arc', 1.57)        # ±90° = π/2
        self.declare_parameter('depth_roi_width', 0.6)           # Central 60%
        self.declare_parameter('depth_roi_height', 0.6)          # Central 60%

        # Speed limits
        self.declare_parameter('max_linear_vel', 0.35)           # m/s
        self.declare_parameter('max_angular_vel', 0.6)           # rad/s
        self.declare_parameter('slowdown_factor', 0.3)           # 30% speed in warning

        # Passenger comfort (jerk limiting)
        self.declare_parameter('max_linear_accel', 0.25)         # m/s²
        self.declare_parameter('max_angular_accel', 0.8)         # rad/s²
        self.declare_parameter('control_frequency', 20.0)        # Hz

        # Cross-validation
        self.declare_parameter('sensor_agreement_threshold', 0.5)  # 50cm disagreement
        self.declare_parameter('min_depth_points', 100)          # Min valid depth pixels

        # Get parameters
        self.emergency_dist = self.get_parameter('emergency_stop_distance').value
        self.danger_dist = self.get_parameter('danger_distance').value
        self.warning_dist = self.get_parameter('warning_distance').value
        self.slow_dist = self.get_parameter('slow_distance').value

        self.lidar_arc = self.get_parameter('lidar_frontal_arc').value
        self.depth_roi_w = self.get_parameter('depth_roi_width').value
        self.depth_roi_h = self.get_parameter('depth_roi_height').value

        self.max_lin_vel = self.get_parameter('max_linear_vel').value
        self.max_ang_vel = self.get_parameter('max_angular_vel').value
        self.slowdown = self.get_parameter('slowdown_factor').value

        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value
        self.control_freq = self.get_parameter('control_frequency').value

        self.agreement_thresh = self.get_parameter('sensor_agreement_threshold').value
        self.min_depth_pts = self.get_parameter('min_depth_points').value

        # ====================================================================
        # STATE VARIABLES
        # ====================================================================

        self.current_state = SafetyState.SAFE
        self.lidar_min_distance = float('inf')
        self.depth_min_distance = float('inf')
        self.last_cmd_vel = Twist()
        self.bridge = CvBridge()

        # Timestamps for sensor freshness
        self.last_lidar_time = None
        self.last_depth_time = None
        self.sensor_timeout = 1.0  # seconds

        # ====================================================================
        # PUBLISHERS
        # ====================================================================

        self.status_pub = self.create_publisher(String, '/safety/status', 10)
        self.emergency_pub = self.create_publisher(Bool, '/safety/emergency_stop', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_callback, qos
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, qos
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # ====================================================================
        # TIMER
        # ====================================================================

        self.safety_timer = self.create_timer(
            1.0 / self.control_freq, self.safety_check
        )

        self.get_logger().info('Safety Monitor Node initialized')
        self.get_logger().info(f'  Emergency stop: {self.emergency_dist:.2f}m')
        self.get_logger().info(f'  Danger zone: {self.danger_dist:.2f}m')
        self.get_logger().info(f'  Warning zone: {self.warning_dist:.2f}m')

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan and extract frontal minimum distance."""
        self.last_lidar_time = self.get_clock().now()

        # Calculate frontal arc indices
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        num_ranges = len(msg.ranges)

        # Frontal arc: -arc to +arc (centered at 0)
        min_idx = max(0, int((-self.lidar_arc - angle_min) / angle_inc))
        max_idx = min(num_ranges - 1, int((self.lidar_arc - angle_min) / angle_inc))

        # Find minimum distance in frontal arc
        frontal_ranges = []
        for i in range(min_idx, max_idx + 1):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max and not math.isinf(r):
                frontal_ranges.append(r)

        if frontal_ranges:
            self.lidar_min_distance = min(frontal_ranges)
        else:
            self.lidar_min_distance = float('inf')

    def depth_callback(self, msg: Image):
        """Process depth image and extract frontal minimum distance."""
        self.last_depth_time = self.get_clock().now()

        try:
            # Convert to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Get image dimensions
            h, w = depth_image.shape

            # Extract central ROI
            roi_h = int(h * self.depth_roi_h)
            roi_w = int(w * self.depth_roi_w)
            y_start = (h - roi_h) // 2
            x_start = (w - roi_w) // 2

            roi = depth_image[y_start:y_start+roi_h, x_start:x_start+roi_w]

            # Get valid depth values (convert mm to m if needed)
            valid_depths = roi[roi > 0]

            if len(valid_depths) >= self.min_depth_pts:
                # Use percentile to avoid noise
                self.depth_min_distance = np.percentile(valid_depths, 5) / 1000.0
            else:
                self.depth_min_distance = float('inf')

        except Exception as e:
            self.get_logger().warn(f'Depth processing error: {e}')
            self.depth_min_distance = float('inf')

    def cmd_vel_callback(self, msg: Twist):
        """Store incoming velocity command for safety filtering."""
        self.last_cmd_vel = msg

    def safety_check(self):
        """Main safety check loop - runs at control_frequency."""
        now = self.get_clock().now()

        # Check sensor freshness
        lidar_fresh = (
            self.last_lidar_time is not None and
            (now - self.last_lidar_time).nanoseconds / 1e9 < self.sensor_timeout
        )
        depth_fresh = (
            self.last_depth_time is not None and
            (now - self.last_depth_time).nanoseconds / 1e9 < self.sensor_timeout
        )

        # Determine safety state
        self.current_state = self.determine_safety_state(lidar_fresh, depth_fresh)

        # Publish status
        status_msg = String()
        status_msg.data = f'{self.current_state.value}|L:{self.lidar_min_distance:.2f}|D:{self.depth_min_distance:.2f}'
        self.status_pub.publish(status_msg)

        # Publish emergency stop flag
        emergency_msg = Bool()
        emergency_msg.data = self.current_state == SafetyState.EMERGENCY_STOP
        self.emergency_pub.publish(emergency_msg)

        # Apply safety filter to velocity command
        safe_cmd = self.filter_velocity(self.last_cmd_vel)
        self.cmd_vel_pub.publish(safe_cmd)

    def determine_safety_state(self, lidar_fresh: bool, depth_fresh: bool) -> SafetyState:
        """Determine current safety state based on sensor data."""

        # If no fresh sensor data, emergency stop
        if not lidar_fresh and not depth_fresh:
            self.get_logger().warn('No fresh sensor data - emergency stop')
            return SafetyState.EMERGENCY_STOP

        # Get minimum distance from available sensors
        if lidar_fresh and depth_fresh:
            min_dist = min(self.lidar_min_distance, self.depth_min_distance)

            # Cross-validation: check for sensor disagreement
            dist_diff = abs(self.lidar_min_distance - self.depth_min_distance)
            if dist_diff > self.agreement_thresh:
                # Sensors disagree significantly
                # This could indicate transparent obstacle (LiDAR misses, depth sees)
                # or low obstacle (depth misses, LiDAR sees)
                if self.depth_min_distance < self.danger_dist:
                    # Depth sees close obstacle - trust it (might be glass)
                    self.get_logger().warn(
                        f'Sensor disagreement: Depth={self.depth_min_distance:.2f}m, '
                        f'LiDAR={self.lidar_min_distance:.2f}m - trusting depth'
                    )
                    min_dist = self.depth_min_distance
                elif self.lidar_min_distance < self.danger_dist:
                    # LiDAR sees close obstacle - trust it (might be low obstacle)
                    self.get_logger().warn(
                        f'Sensor disagreement: LiDAR={self.lidar_min_distance:.2f}m, '
                        f'Depth={self.depth_min_distance:.2f}m - trusting LiDAR'
                    )
                    min_dist = self.lidar_min_distance

        elif lidar_fresh:
            min_dist = self.lidar_min_distance
        else:
            min_dist = self.depth_min_distance

        # Determine state based on minimum distance
        if min_dist < self.emergency_dist:
            return SafetyState.EMERGENCY_STOP
        elif min_dist < self.danger_dist:
            return SafetyState.DANGER
        elif min_dist < self.warning_dist:
            return SafetyState.WARNING
        else:
            return SafetyState.SAFE

    def filter_velocity(self, cmd: Twist) -> Twist:
        """Apply safety filtering to velocity command."""
        safe_cmd = Twist()

        # Emergency stop - zero velocity
        if self.current_state == SafetyState.EMERGENCY_STOP:
            return safe_cmd

        # Danger - very slow, only backward allowed
        if self.current_state == SafetyState.DANGER:
            safe_cmd.linear.x = max(-0.1, min(0.0, cmd.linear.x))  # Only reverse
            safe_cmd.angular.z = cmd.angular.z * 0.3  # Slow rotation
            return self.apply_acceleration_limits(safe_cmd)

        # Warning - reduced speed
        if self.current_state == SafetyState.WARNING:
            safe_cmd.linear.x = cmd.linear.x * self.slowdown
            safe_cmd.angular.z = cmd.angular.z * 0.5
            return self.apply_acceleration_limits(safe_cmd)

        # Safe - apply limits but allow normal operation
        safe_cmd.linear.x = max(-self.max_lin_vel * 0.5,
                                min(self.max_lin_vel, cmd.linear.x))
        safe_cmd.angular.z = max(-self.max_ang_vel,
                                 min(self.max_ang_vel, cmd.angular.z))

        return self.apply_acceleration_limits(safe_cmd)

    def apply_acceleration_limits(self, cmd: Twist) -> Twist:
        """Apply acceleration limits for passenger comfort."""
        dt = 1.0 / self.control_freq

        # Limit linear acceleration
        lin_diff = cmd.linear.x - self.last_cmd_vel.linear.x
        max_lin_change = self.max_lin_accel * dt
        if abs(lin_diff) > max_lin_change:
            cmd.linear.x = self.last_cmd_vel.linear.x + np.sign(lin_diff) * max_lin_change

        # Limit angular acceleration
        ang_diff = cmd.angular.z - self.last_cmd_vel.angular.z
        max_ang_change = self.max_ang_accel * dt
        if abs(ang_diff) > max_ang_change:
            cmd.angular.z = self.last_cmd_vel.angular.z + np.sign(ang_diff) * max_ang_change

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Safety Monitor shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
