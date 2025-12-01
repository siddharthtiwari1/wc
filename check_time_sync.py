#!/usr/bin/env python3
"""
Time Synchronization Diagnostic for SLAM Drift Analysis

Checks timestamp alignment between:
- /scan (LIDAR)
- /wc_control/odom (wheel odometry)
- /odometry/filtered (EKF output)
- /imu (IMU data)
- TF transforms

Run: python3 check_time_sync.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from collections import deque
import time


class TimeSyncChecker(Node):
    def __init__(self):
        super().__init__('time_sync_checker')

        # Store last timestamps
        self.timestamps = {
            '/scan': None,
            '/wc_control/odom': None,
            '/odometry/filtered': None,
            '/imu': None,
        }

        # Store message rates
        self.rates = {k: deque(maxlen=50) for k in self.timestamps}
        self.last_times = {k: None for k in self.timestamps}

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.create_subscription(Odometry, '/wc_control/odom', self.wc_odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, sensor_qos)

        # Timer for reporting
        self.create_timer(2.0, self.report_sync)

        self.get_logger().info('Time Sync Checker started - waiting for messages...')

    def _update_timestamp(self, topic, msg):
        now_ros = self.get_clock().now()
        msg_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        self.timestamps[topic] = {
            'msg_stamp': msg_stamp,
            'recv_time': now_ros,
            'latency_ms': (now_ros - msg_stamp).nanoseconds / 1e6,
            'frame_id': msg.header.frame_id
        }

        # Calculate rate
        if self.last_times[topic] is not None:
            dt = (now_ros - self.last_times[topic]).nanoseconds / 1e9
            if dt > 0:
                self.rates[topic].append(1.0 / dt)
        self.last_times[topic] = now_ros

    def scan_cb(self, msg):
        self._update_timestamp('/scan', msg)

    def wc_odom_cb(self, msg):
        self._update_timestamp('/wc_control/odom', msg)

    def ekf_odom_cb(self, msg):
        self._update_timestamp('/odometry/filtered', msg)

    def imu_cb(self, msg):
        self._update_timestamp('/imu', msg)

    def report_sync(self):
        now = self.get_clock().now()

        print("\n" + "="*80)
        print("TIME SYNCHRONIZATION REPORT")
        print("="*80)
        print(f"{'Topic':<25} {'Latency(ms)':<12} {'Rate(Hz)':<10} {'Frame':<20} {'Status'}")
        print("-"*80)

        active_stamps = []

        for topic, data in self.timestamps.items():
            if data is None:
                print(f"{topic:<25} {'NO DATA':<12} {'--':<10} {'--':<20} MISSING")
                continue

            latency = data['latency_ms']
            rate = sum(self.rates[topic]) / len(self.rates[topic]) if self.rates[topic] else 0
            frame = data['frame_id']

            # Check if data is stale (>1 second old)
            age = (now - data['recv_time']).nanoseconds / 1e9
            if age > 1.0:
                status = f"STALE ({age:.1f}s)"
            elif abs(latency) > 100:
                status = "HIGH LATENCY"
            elif latency < -50:
                status = "FUTURE STAMP"
            else:
                status = "OK"

            active_stamps.append((topic, data['msg_stamp']))
            print(f"{topic:<25} {latency:>8.1f}    {rate:>6.1f}    {frame:<20} {status}")

        # Check relative timing between topics
        if len(active_stamps) >= 2:
            print("\n" + "-"*80)
            print("RELATIVE TIMING (for SLAM - scan vs odom alignment):")
            print("-"*80)

            scan_data = self.timestamps.get('/scan')
            odom_data = self.timestamps.get('/wc_control/odom')
            ekf_data = self.timestamps.get('/odometry/filtered')
            imu_data = self.timestamps.get('/imu')

            if scan_data and odom_data:
                diff_ms = (scan_data['msg_stamp'] - odom_data['msg_stamp']).nanoseconds / 1e6
                status = "OK" if abs(diff_ms) < 50 else "DRIFT RISK" if abs(diff_ms) < 100 else "CRITICAL"
                print(f"  /scan vs /wc_control/odom:     {diff_ms:>8.1f} ms  [{status}]")

            if scan_data and ekf_data:
                diff_ms = (scan_data['msg_stamp'] - ekf_data['msg_stamp']).nanoseconds / 1e6
                status = "OK" if abs(diff_ms) < 50 else "DRIFT RISK" if abs(diff_ms) < 100 else "CRITICAL"
                print(f"  /scan vs /odometry/filtered:   {diff_ms:>8.1f} ms  [{status}]")

            if odom_data and imu_data:
                diff_ms = (odom_data['msg_stamp'] - imu_data['msg_stamp']).nanoseconds / 1e6
                status = "OK" if abs(diff_ms) < 30 else "EKF FUSION ISSUE" if abs(diff_ms) < 100 else "CRITICAL"
                print(f"  /wc_control/odom vs /imu:      {diff_ms:>8.1f} ms  [{status}]")

        # Check TF timing
        print("\n" + "-"*80)
        print("TF TRANSFORM TIMING:")
        print("-"*80)

        tf_checks = [
            ('odom', 'base_link'),
            ('map', 'odom'),
            ('base_link', 'rplidar_link'),
        ]

        for parent, child in tf_checks:
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                tf_stamp = rclpy.time.Time.from_msg(tf.header.stamp)
                age_ms = (now - tf_stamp).nanoseconds / 1e6
                status = "OK" if age_ms < 100 else "STALE" if age_ms < 500 else "VERY STALE"
                print(f"  {parent} -> {child}: age = {age_ms:.0f} ms [{status}]")
            except Exception as e:
                print(f"  {parent} -> {child}: NOT AVAILABLE ({type(e).__name__})")

        print("="*80)


def main():
    rclpy.init()
    node = TimeSyncChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
