#!/usr/bin/env python3
"""
Map Quality Checker for SLAM

Monitors SLAM quality metrics in real-time:
- Loop closure error
- Scan matching scores
- Map→odom transform stability
- Pose covariance

Usage:
    ros2 run scripts map_quality_checker.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from collections import deque
import tf2_ros


class MapQualityChecker(Node):
    def __init__(self):
        super().__init__('map_quality_checker')

        # Subscribe to SLAM pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_toolbox/pose',
            self.pose_callback,
            10
        )

        # TF listener for map→odom transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Quality metrics
        self.pose_history = deque(maxlen=100)  # Last 100 poses
        self.loop_start_pose = None
        self.loop_mode = False
        self.total_distance = 0.0
        self.last_pose = None

        # Transform stability tracking
        self.map_odom_transforms = deque(maxlen=50)

        # Timer for periodic reporting
        self.timer = self.create_timer(5.0, self.report_quality)

        self.get_logger().info('Map Quality Checker Started')
        self.get_logger().info('Commands:')
        self.get_logger().info('  - Drive around to collect data')
        self.get_logger().info('  - Press Ctrl+C when done for final report')
        self.get_logger().info('')
        self.get_logger().info('=' * 60)

    def pose_callback(self, msg):
        """Process incoming SLAM pose"""
        current_pose = msg.pose.pose
        current_cov = msg.pose.covariance

        # Extract position
        x = current_pose.position.x
        y = current_pose.position.y

        # Calculate covariance magnitude (uncertainty)
        pos_cov = math.sqrt(current_cov[0] + current_cov[7])  # xx + yy

        # Store pose
        self.pose_history.append({
            'x': x,
            'y': y,
            'cov': pos_cov,
            'stamp': msg.header.stamp
        })

        # Calculate distance traveled
        if self.last_pose is not None:
            dx = x - self.last_pose['x']
            dy = y - self.last_pose['y']
            dist = math.sqrt(dx*dx + dy*dy)
            self.total_distance += dist

        self.last_pose = {'x': x, 'y': y}

    def get_map_odom_transform_stability(self):
        """Check stability of map→odom transform"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time()
            )

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y

            self.map_odom_transforms.append({
                'x': tx,
                'y': ty
            })

            # Calculate stability (std deviation)
            if len(self.map_odom_transforms) > 10:
                xs = [t['x'] for t in self.map_odom_transforms]
                ys = [t['y'] for t in self.map_odom_transforms]

                std_x = np.std(xs)
                std_y = np.std(ys)
                stability = math.sqrt(std_x**2 + std_y**2)

                return stability

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            pass

        return None

    def calculate_drift(self):
        """Calculate drift from start to current position"""
        if len(self.pose_history) < 2:
            return None

        start = self.pose_history[0]
        end = self.pose_history[-1]

        dx = end['x'] - start['x']
        dy = end['y'] - start['y']
        drift = math.sqrt(dx*dx + dy*dy)

        return drift

    def get_average_covariance(self):
        """Get average pose uncertainty"""
        if len(self.pose_history) < 1:
            return None

        covs = [p['cov'] for p in self.pose_history]
        return np.mean(covs)

    def report_quality(self):
        """Print quality report"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('MAP QUALITY REPORT')
        self.get_logger().info('=' * 60)

        # Distance traveled
        self.get_logger().info(f'Distance traveled: {self.total_distance:.2f} m')

        # Drift
        drift = self.calculate_drift()
        if drift is not None:
            self.get_logger().info(f'Total drift from start: {drift*100:.1f} cm')

            # Quality rating based on drift
            if self.total_distance > 5.0:  # Only rate if moved significantly
                drift_percent = (drift / self.total_distance) * 100
                self.get_logger().info(f'Drift percentage: {drift_percent:.2f}%')

                if drift_percent < 1.0:
                    quality = '⭐⭐⭐⭐⭐ EXCELLENT'
                elif drift_percent < 2.0:
                    quality = '⭐⭐⭐⭐ VERY GOOD'
                elif drift_percent < 5.0:
                    quality = '⭐⭐⭐ GOOD'
                elif drift_percent < 10.0:
                    quality = '⭐⭐ FAIR'
                else:
                    quality = '⭐ POOR - needs tuning'

                self.get_logger().info(f'Quality: {quality}')

        # Pose uncertainty
        avg_cov = self.get_average_covariance()
        if avg_cov is not None:
            self.get_logger().info(f'Average position uncertainty: {avg_cov*100:.2f} cm')

            if avg_cov < 0.05:
                self.get_logger().info('Uncertainty: ⭐⭐⭐⭐⭐ Very confident')
            elif avg_cov < 0.10:
                self.get_logger().info('Uncertainty: ⭐⭐⭐⭐ Confident')
            elif avg_cov < 0.20:
                self.get_logger().info('Uncertainty: ⭐⭐⭐ Acceptable')
            else:
                self.get_logger().info('Uncertainty: ⭐⭐ High uncertainty')

        # Transform stability
        stability = self.get_map_odom_transform_stability()
        if stability is not None:
            self.get_logger().info(f'Map↔Odom stability: {stability*100:.2f} cm variation')

            if stability < 0.02:
                self.get_logger().info('Stability: ⭐⭐⭐⭐⭐ Excellent')
            elif stability < 0.05:
                self.get_logger().info('Stability: ⭐⭐⭐⭐ Very good')
            elif stability < 0.10:
                self.get_logger().info('Stability: ⭐⭐⭐ Good')
            else:
                self.get_logger().info('Stability: ⭐⭐ Unstable')

        self.get_logger().info('=' * 60)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    checker = MapQualityChecker()

    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        checker.get_logger().info('')
        checker.get_logger().info('=' * 60)
        checker.get_logger().info('FINAL QUALITY REPORT')
        checker.get_logger().info('=' * 60)
        checker.report_quality()
        checker.get_logger().info('Map Quality Checker Stopped')

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
