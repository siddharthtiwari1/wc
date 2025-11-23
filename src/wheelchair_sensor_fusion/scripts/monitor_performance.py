#!/usr/bin/env python3
"""
Performance Monitoring Tool for Wheelchair Sensor Fusion

Monitors real-time performance metrics:
- Topic publishing rates (Hz)
- Message latency
- CPU and GPU usage
- Memory consumption
- Fusion success rate
- Operating mode stability

Author: Siddharth Tiwari (IIT Mandi)
For: ICRA 2025 Submission
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection2DArray

import time
import psutil
import os
from collections import deque
from typing import Dict, List
import subprocess


class PerformanceMonitor(Node):
    """Monitors system performance metrics."""

    def __init__(self):
        super().__init__('performance_monitor')

        # Topic statistics
        self.topic_stats = {
            '/scan': {'count': 0, 'last_time': None, 'rates': deque(maxlen=30)},
            '/camera/color/image_raw': {'count': 0, 'last_time': None, 'rates': deque(maxlen=30)},
            '/yolo/detections': {'count': 0, 'last_time': None, 'rates': deque(maxlen=30)},
            '/fusion/obstacles': {'count': 0, 'last_time': None, 'rates': deque(maxlen=30)},
        }

        # Latency tracking (sensor input to fusion output)
        self.sensor_timestamps = {}
        self.fusion_latencies = deque(maxlen=100)

        # Fusion diagnostics
        self.current_mode = 'unknown'
        self.mode_history = deque(maxlen=100)
        self.last_diagnostics = None

        # Create subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.yolo_cb, 10)
        self.create_subscription(MarkerArray, '/fusion/obstacles', self.fusion_cb, 10)
        self.create_subscription(String, '/fusion/status', self.status_cb, 10)
        self.create_subscription(String, '/fusion/diagnostics', self.diagnostics_cb, 10)

        # Timer for reporting
        self.report_timer = self.create_timer(5.0, self.report_performance)

        # System process monitoring
        self.process_pid = os.getpid()
        self.process = psutil.Process(self.process_pid)

        self.get_logger().info('Performance monitor started')

    def scan_cb(self, msg):
        """LiDAR scan callback."""
        self.update_topic_stats('/scan', msg.header.stamp)
        self.sensor_timestamps['scan'] = self.get_clock().now()

    def image_cb(self, msg):
        """Camera image callback."""
        self.update_topic_stats('/camera/color/image_raw', msg.header.stamp)
        self.sensor_timestamps['camera'] = self.get_clock().now()

    def yolo_cb(self, msg):
        """YOLO detections callback."""
        self.update_topic_stats('/yolo/detections', None)

    def fusion_cb(self, msg):
        """Fusion obstacles callback."""
        self.update_topic_stats('/fusion/obstacles', None)

        # Calculate latency from last sensor input
        if 'scan' in self.sensor_timestamps or 'camera' in self.sensor_timestamps:
            now = self.get_clock().now()
            latencies = []

            if 'scan' in self.sensor_timestamps:
                latency_scan = (now - self.sensor_timestamps['scan']).nanoseconds / 1e6  # ms
                latencies.append(latency_scan)

            if 'camera' in self.sensor_timestamps:
                latency_cam = (now - self.sensor_timestamps['camera']).nanoseconds / 1e6  # ms
                latencies.append(latency_cam)

            if latencies:
                self.fusion_latencies.append(max(latencies))  # Worst-case latency

    def status_cb(self, msg):
        """Fusion status callback."""
        # Parse mode from status string (format: "mode=full_fusion,obstacles=3")
        if 'mode=' in msg.data:
            parts = msg.data.split(',')
            for part in parts:
                if part.startswith('mode='):
                    mode = part.split('=')[1]
                    if mode != self.current_mode:
                        self.current_mode = mode
                        self.mode_history.append((self.get_clock().now(), mode))

    def diagnostics_cb(self, msg):
        """Fusion diagnostics callback."""
        self.last_diagnostics = msg.data

    def update_topic_stats(self, topic: str, timestamp=None):
        """Update statistics for a topic."""
        if topic not in self.topic_stats:
            return

        stats = self.topic_stats[topic]
        stats['count'] += 1

        current_time = self.get_clock().now()
        if stats['last_time'] is not None:
            delta = (current_time - stats['last_time']).nanoseconds / 1e9  # seconds
            if delta > 0:
                rate = 1.0 / delta
                stats['rates'].append(rate)

        stats['last_time'] = current_time

    def get_topic_rate(self, topic: str) -> float:
        """Get average rate for a topic."""
        if topic not in self.topic_stats:
            return 0.0

        rates = self.topic_stats[topic]['rates']
        if len(rates) == 0:
            return 0.0

        return sum(rates) / len(rates)

    def get_system_stats(self) -> Dict:
        """Get system resource usage."""
        stats = {
            'cpu_percent': self.process.cpu_percent(interval=0.1),
            'memory_mb': self.process.memory_info().rss / 1024 / 1024,
            'num_threads': self.process.num_threads(),
        }

        # Try to get GPU stats
        try:
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used',
                                     '--format=csv,noheader,nounits'],
                                    capture_output=True, text=True, timeout=1)
            if result.returncode == 0:
                gpu_util, gpu_mem = result.stdout.strip().split(',')
                stats['gpu_percent'] = float(gpu_util.strip())
                stats['gpu_memory_mb'] = float(gpu_mem.strip())
        except Exception:
            stats['gpu_percent'] = None
            stats['gpu_memory_mb'] = None

        return stats

    def report_performance(self):
        """Report current performance metrics."""
        self.get_logger().info('=' * 80)
        self.get_logger().info('PERFORMANCE REPORT')
        self.get_logger().info('=' * 80)

        # Topic rates
        self.get_logger().info('Topic Rates (Hz):')
        for topic in self.topic_stats.keys():
            rate = self.get_topic_rate(topic)
            status = self.rate_status(topic, rate)
            self.get_logger().info(f'  {topic:40s}: {rate:6.2f} Hz {status}')

        # Latency
        if len(self.fusion_latencies) > 0:
            avg_latency = sum(self.fusion_latencies) / len(self.fusion_latencies)
            max_latency = max(self.fusion_latencies)
            min_latency = min(self.fusion_latencies)
            self.get_logger().info('')
            self.get_logger().info('Fusion Latency:')
            self.get_logger().info(f'  Average: {avg_latency:6.2f} ms')
            self.get_logger().info(f'  Min:     {min_latency:6.2f} ms')
            self.get_logger().info(f'  Max:     {max_latency:6.2f} ms')

            latency_status = '✓ GOOD' if avg_latency < 50 else '⚠ HIGH' if avg_latency < 100 else '✗ CRITICAL'
            self.get_logger().info(f'  Status:  {latency_status}')

        # Operating mode
        self.get_logger().info('')
        self.get_logger().info(f'Current Mode: {self.current_mode.upper()}')
        if len(self.mode_history) > 1:
            self.get_logger().info(f'Mode changes in last 100 samples: {len(self.mode_history)}')

        # System resources
        sys_stats = self.get_system_stats()
        self.get_logger().info('')
        self.get_logger().info('System Resources:')
        self.get_logger().info(f'  CPU:      {sys_stats["cpu_percent"]:6.2f} %')
        self.get_logger().info(f'  Memory:   {sys_stats["memory_mb"]:6.2f} MB')
        self.get_logger().info(f'  Threads:  {sys_stats["num_threads"]}')

        if sys_stats['gpu_percent'] is not None:
            self.get_logger().info(f'  GPU:      {sys_stats["gpu_percent"]:6.2f} %')
            self.get_logger().info(f'  GPU Mem:  {sys_stats["gpu_memory_mb"]:6.2f} MB')

        # Diagnostics
        if self.last_diagnostics:
            self.get_logger().info('')
            self.get_logger().info('Last Diagnostics:')
            for line in self.last_diagnostics.split('\n')[:10]:  # First 10 lines
                if line.strip():
                    self.get_logger().info(f'  {line}')

        self.get_logger().info('=' * 80)

    def rate_status(self, topic: str, rate: float) -> str:
        """Get status symbol for a topic rate."""
        # Expected rates
        expected = {
            '/scan': 20.0,  # RPLidar S3 @ 20 Hz
            '/camera/color/image_raw': 25.0,  # RealSense @ 30 Hz (allow margin)
            '/yolo/detections': 25.0,  # YOLO should be ~25-30 Hz on GPU
            '/fusion/obstacles': 25.0,  # Fusion should be ~25-30 Hz
        }

        if topic not in expected:
            return ''

        target = expected[topic]

        if rate >= target * 0.9:  # Within 90% of expected
            return '✓'
        elif rate >= target * 0.5:  # At least 50% of expected
            return '⚠'
        else:
            return '✗'


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
