#!/usr/bin/env python3
"""
Loop Closure Quality Test

Interactive test to measure SLAM loop closure accuracy.
Guides user through driving a square and calculates closure error.

Usage:
    ros2 run scripts loop_closure_test.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time


class LoopClosureTest(Node):
    def __init__(self):
        super().__init__('loop_closure_test')

        # Subscribe to SLAM pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_toolbox/pose',
            self.pose_callback,
            10
        )

        self.current_pose = None
        self.start_pose = None
        self.test_active = False

        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('LOOP CLOSURE QUALITY TEST')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('This test measures how accurately SLAM closes a loop.')
        self.get_logger().info('')
        self.get_logger().info('Instructions:')
        self.get_logger().info('1. Place wheelchair at starting position (mark with tape)')
        self.get_logger().info('2. Press ENTER when ready to start test')
        self.get_logger().info('3. Drive a square pattern:')
        self.get_logger().info('   - Forward 3m → Rotate 90° left')
        self.get_logger().info('   - Forward 3m → Rotate 90° left')
        self.get_logger().info('   - Forward 3m → Rotate 90° left')
        self.get_logger().info('   - Forward 3m → Rotate 90° left (back to start)')
        self.get_logger().info('4. Return to the tape mark')
        self.get_logger().info('5. Press ENTER to finish test')
        self.get_logger().info('')
        self.get_logger().info('=' * 70)

        # Wait for pose data
        self.get_logger().info('Waiting for SLAM pose data...')
        while self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('✓ Receiving pose data')
        self.get_logger().info('')

        # Interactive test
        self.run_test()

    def pose_callback(self, msg):
        """Store current pose"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def run_test(self):
        """Run interactive loop closure test"""
        # Wait for user to start
        input('Press ENTER to record starting position...')

        # Record start
        rclpy.spin_once(self, timeout_sec=0.1)
        self.start_pose = self.current_pose.copy()

        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('STARTING POSITION RECORDED')
        self.get_logger().info(f'  X: {self.start_pose["x"]:.3f} m')
        self.get_logger().info(f'  Y: {self.start_pose["y"]:.3f} m')
        self.get_logger().info(f'  Theta: {math.degrees(self.start_pose["theta"]):.1f}°')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')

        self.get_logger().info('Now drive the square pattern...')
        self.get_logger().info('(Forward 3m, turn 90° left) x 4 times')
        self.get_logger().info('')

        # Real-time monitoring
        self.get_logger().info('Monitoring position (press Ctrl+C to finish)...')
        self.get_logger().info('')

        last_report = time.time()
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.1)

                # Report current position every 2 seconds
                if time.time() - last_report > 2.0:
                    dx = self.current_pose['x'] - self.start_pose['x']
                    dy = self.current_pose['y'] - self.start_pose['y']
                    dist = math.sqrt(dx*dx + dy*dy)

                    self.get_logger().info(
                        f'Distance from start: {dist*100:.1f} cm '
                        f'(X: {dx*100:+.1f} cm, Y: {dy*100:+.1f} cm)'
                    )
                    last_report = time.time()

        except KeyboardInterrupt:
            self.get_logger().info('')
            self.get_logger().info('Test stopped by user')

        # Calculate final error
        self.calculate_results()

    def calculate_results(self):
        """Calculate and display loop closure error"""
        rclpy.spin_once(self, timeout_sec=0.1)
        end_pose = self.current_pose

        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('LOOP CLOSURE TEST RESULTS')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')

        # Position error
        dx = end_pose['x'] - self.start_pose['x']
        dy = end_pose['y'] - self.start_pose['y']
        position_error = math.sqrt(dx*dx + dy*dy)

        # Angle error
        dtheta = end_pose['theta'] - self.start_pose['theta']
        # Normalize to [-pi, pi]
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi
        angle_error = abs(dtheta)

        self.get_logger().info('Starting Position:')
        self.get_logger().info(f'  X: {self.start_pose["x"]:.3f} m')
        self.get_logger().info(f'  Y: {self.start_pose["y"]:.3f} m')
        self.get_logger().info(f'  Theta: {math.degrees(self.start_pose["theta"]):.1f}°')
        self.get_logger().info('')

        self.get_logger().info('Ending Position:')
        self.get_logger().info(f'  X: {end_pose["x"]:.3f} m')
        self.get_logger().info(f'  Y: {end_pose["y"]:.3f} m')
        self.get_logger().info(f'  Theta: {math.degrees(end_pose["theta"]):.1f}°')
        self.get_logger().info('')

        self.get_logger().info('Closure Error:')
        self.get_logger().info(f'  X error: {abs(dx)*100:.2f} cm')
        self.get_logger().info(f'  Y error: {abs(dy)*100:.2f} cm')
        self.get_logger().info(f'  Position error: {position_error*100:.2f} cm')
        self.get_logger().info(f'  Angle error: {math.degrees(angle_error):.2f}°')
        self.get_logger().info('')

        # Quality rating
        self.get_logger().info('-' * 70)
        self.get_logger().info('QUALITY RATING:')
        self.get_logger().info('-' * 70)

        if position_error < 0.05:  # < 5 cm
            rating = '⭐⭐⭐⭐⭐ EXCELLENT'
            comment = 'Outstanding loop closure! Map quality is superb.'
        elif position_error < 0.10:  # 5-10 cm
            rating = '⭐⭐⭐⭐ VERY GOOD'
            comment = 'Very good loop closure. Map quality is high.'
        elif position_error < 0.20:  # 10-20 cm
            rating = '⭐⭐⭐ GOOD'
            comment = 'Good loop closure. Map is usable for navigation.'
        elif position_error < 0.50:  # 20-50 cm
            rating = '⭐⭐ FAIR'
            comment = 'Fair loop closure. Consider tuning parameters.'
        else:  # > 50 cm
            rating = '⭐ POOR'
            comment = 'Poor loop closure. Check SLAM config and odometry quality.'

        self.get_logger().info(f'{rating}')
        self.get_logger().info(f'{comment}')
        self.get_logger().info('')

        # Recommendations
        if position_error > 0.20:
            self.get_logger().info('RECOMMENDATIONS:')
            self.get_logger().info('  - Check odometry quality (wheel encoders)')
            self.get_logger().info('  - Increase minimum_travel_distance (more movement between scans)')
            self.get_logger().info('  - Check if lidar is inverted correctly')
            self.get_logger().info('  - Ensure environment has good features (corners, walls)')
            self.get_logger().info('')

        self.get_logger().info('=' * 70)


def main(args=None):
    rclpy.init(args=args)
    test = LoopClosureTest()
    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
