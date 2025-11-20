#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from datetime import datetime


class SafetyMonitor(Node):
    """
    🛡️ WHEELCHAIR SAFETY MONITOR

    Monitors wheelchair safety parameters and can trigger emergency stops.
    Tracks velocity limits, obstacle detection, and system faults.
    """

    def __init__(self):
        super().__init__('safety_monitor')

        # Safety parameters
        self.declare_parameter('max_linear_velocity', 2.68)  # m/s
        self.declare_parameter('max_angular_velocity', 17.59)  # rad/s
        self.declare_parameter('min_obstacle_distance', 0.3)  # meters
        self.declare_parameter('velocity_timeout', 2.0)  # seconds
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')

        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.min_obstacle_dist = self.get_parameter('min_obstacle_distance').value
        self.velocity_timeout = self.get_parameter('velocity_timeout').value
        self.emergency_topic = self.get_parameter('emergency_stop_topic').value

        # Safety state
        self.emergency_stop_active = False
        self.safety_violations = []
        self.last_cmd_vel_time = None
        self.current_velocity = None
        self.obstacles_detected = False
        self.min_laser_distance = float('inf')

        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool, self.emergency_topic, qos_reliable)
        self.safety_status_pub = self.create_publisher(
            String, '/safety_status', qos_reliable)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/wc_control/cmd_vel', self.cmd_vel_callback, qos_best_effort)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, qos_best_effort)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_best_effort)

        # Safety check timer (10 Hz)
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        # Status reporting timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_safety_status)

        self.get_logger().info('🛡️ Safety Monitor Initialized')
        self.get_logger().info(f'📏 Max Linear Velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'🔄 Max Angular Velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'🚧 Min Obstacle Distance: {self.min_obstacle_dist} m')

    def cmd_vel_callback(self, msg):
        """Monitor command velocity for safety violations"""
        self.last_cmd_vel_time = time.time()

        # Check velocity limits
        linear_speed = abs(msg.linear.x)
        angular_speed = abs(msg.angular.z)

        violations = []

        if linear_speed > self.max_linear_vel:
            violations.append(f'Linear velocity exceeded: {linear_speed:.2f} > {self.max_linear_vel}')

        if angular_speed > self.max_angular_vel:
            violations.append(f'Angular velocity exceeded: {angular_speed:.2f} > {self.max_angular_vel}')

        # Check for obstacles in direction of motion
        if self.obstacles_detected and linear_speed > 0.1:
            violations.append(f'Moving towards obstacle: {self.min_laser_distance:.2f}m')

        if violations:
            self.trigger_safety_violation(violations)

    def odom_callback(self, msg):
        """Monitor actual robot velocity"""
        self.current_velocity = msg.twist.twist

    def laser_callback(self, msg):
        """Monitor laser scan for obstacles"""
        if not msg.ranges:
            return

        # Find minimum distance in front sector (±30 degrees)
        front_ranges = []
        total_points = len(msg.ranges)
        front_sector = int(total_points * 0.167)  # ±30 degrees ≈ 16.7% of 360°

        # Front sector: center ± front_sector points
        center = total_points // 2
        start_idx = max(0, center - front_sector)
        end_idx = min(total_points, center + front_sector)

        for i in range(start_idx, end_idx):
            if msg.range_min <= msg.ranges[i] <= msg.range_max:
                front_ranges.append(msg.ranges[i])

        if front_ranges:
            self.min_laser_distance = min(front_ranges)
            self.obstacles_detected = self.min_laser_distance < self.min_obstacle_dist
        else:
            self.min_laser_distance = float('inf')
            self.obstacles_detected = False

    def safety_check(self):
        """Perform periodic safety checks"""
        current_time = time.time()
        violations = []

        # Check for command velocity timeout
        if (self.last_cmd_vel_time is not None and
            current_time - self.last_cmd_vel_time > self.velocity_timeout):
            violations.append('Command velocity timeout - no recent commands')

        # Check current velocity vs limits (if we have odometry)
        if self.current_velocity is not None:
            actual_linear = abs(self.current_velocity.linear.x)
            actual_angular = abs(self.current_velocity.angular.z)

            if actual_linear > self.max_linear_vel * 1.1:  # 10% tolerance
                violations.append(f'Actual linear velocity exceeded: {actual_linear:.2f}')

            if actual_angular > self.max_angular_vel * 1.1:  # 10% tolerance
                violations.append(f'Actual angular velocity exceeded: {actual_angular:.2f}')

        # Update safety violations
        self.safety_violations = violations

        # Trigger emergency stop if critical violations
        if violations and not self.emergency_stop_active:
            critical_violations = [v for v in violations if 'exceeded' in v or 'obstacle' in v]
            if critical_violations:
                self.trigger_emergency_stop(critical_violations)

    def trigger_safety_violation(self, violations):
        """Log safety violations"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        for violation in violations:
            self.get_logger().warn(f'⚠️ [{timestamp}] SAFETY VIOLATION: {violation}')

        self.safety_violations.extend(violations)

    def trigger_emergency_stop(self, violations):
        """Trigger emergency stop"""
        if self.emergency_stop_active:
            return

        self.emergency_stop_active = True
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        self.get_logger().error('🚨' * 20)
        self.get_logger().error(f'🛑 [{timestamp}] EMERGENCY STOP ACTIVATED!')

        for violation in violations:
            self.get_logger().error(f'🔴 REASON: {violation}')

        self.get_logger().error('🚨' * 20)

        # Publish emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)

    def reset_emergency_stop(self):
        """Reset emergency stop (manual intervention required)"""
        self.emergency_stop_active = False
        self.safety_violations.clear()

        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_stop_pub.publish(emergency_msg)

        self.get_logger().info('✅ Emergency stop reset')

    def publish_safety_status(self):
        """Publish current safety status"""
        status_msg = String()

        if self.emergency_stop_active:
            status = "EMERGENCY_STOP"
        elif self.safety_violations:
            status = "VIOLATIONS_DETECTED"
        elif self.obstacles_detected:
            status = "OBSTACLES_DETECTED"
        else:
            status = "SAFE"

        # Create detailed status
        status_details = {
            'status': status,
            'emergency_stop': self.emergency_stop_active,
            'obstacles': self.obstacles_detected,
            'min_distance': f'{self.min_laser_distance:.2f}m',
            'violations': len(self.safety_violations),
            'timestamp': datetime.now().isoformat()
        }

        status_msg.data = str(status_details)
        self.safety_status_pub.publish(status_msg)

        # Log status periodically
        if self.emergency_stop_active or self.safety_violations:
            self.get_logger().info(f'🛡️ Safety Status: {status}')
            if self.obstacles_detected:
                self.get_logger().info(f'🚧 Nearest obstacle: {self.min_laser_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)

    try:
        safety_monitor = SafetyMonitor()
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in safety monitor: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()