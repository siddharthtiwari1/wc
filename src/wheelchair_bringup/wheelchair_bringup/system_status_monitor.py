#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from datetime import datetime


class SystemStatusMonitor(Node):
    """
    🖥️ WHEELCHAIR SYSTEM STATUS MONITOR

    Monitors all critical topics and reports system health.
    Provides real-time status of sensors, control, and navigation.
    """

    def __init__(self):
        super().__init__('system_status_monitor')

        # Parameters
        self.declare_parameter('monitor_topics', [
            '/joint_states',
            '/camera/color/image_raw',
            '/scan',
            '/odometry/filtered',
            '/wc_control/cmd_vel'
        ])
        self.declare_parameter('status_interval', 5.0)  # seconds

        self.monitor_topics = self.get_parameter('monitor_topics').value
        self.status_interval = self.get_parameter('status_interval').value

        # Topic status tracking
        self.topic_status = {}
        self.last_received = {}

        # Initialize status for each topic
        for topic in self.monitor_topics:
            self.topic_status[topic] = 'UNKNOWN'
            self.last_received[topic] = None

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Create subscribers for each monitored topic
        self.create_subscription(JointState, '/joint_states',
                               lambda msg: self.update_status('/joint_states', msg.header), qos_profile)
        self.create_subscription(Image, '/camera/color/image_raw',
                               lambda msg: self.update_status('/camera/color/image_raw', msg.header), qos_profile)
        self.create_subscription(LaserScan, '/scan',
                               lambda msg: self.update_status('/scan', msg.header), qos_profile)
        self.create_subscription(Odometry, '/odometry/filtered',
                               lambda msg: self.update_status('/odometry/filtered', msg.header), qos_profile)
        self.create_subscription(Twist, '/wc_control/cmd_vel',
                               lambda msg: self.update_status('/wc_control/cmd_vel', None), qos_profile)

        # Status reporting timer
        self.status_timer = self.create_timer(self.status_interval, self.report_status)

        # System startup time
        self.startup_time = time.time()

        self.get_logger().info('🖥️ System Status Monitor Started')
        self.get_logger().info(f'Monitoring topics: {self.monitor_topics}')

    def update_status(self, topic_name, header):
        """Update status for a specific topic"""
        current_time = time.time()
        self.last_received[topic_name] = current_time

        # Check message freshness
        if header is not None:
            msg_time = header.stamp.sec + header.stamp.nanosec * 1e-9
            current_ros_time = self.get_clock().now().nanoseconds * 1e-9
            time_diff = abs(current_ros_time - msg_time)

            if time_diff < 1.0:  # Message is fresh (< 1 second old)
                self.topic_status[topic_name] = 'ACTIVE'
            else:
                self.topic_status[topic_name] = 'STALE'
        else:
            # For topics without header (like Twist)
            self.topic_status[topic_name] = 'ACTIVE'

    def report_status(self):
        """Report comprehensive system status"""
        current_time = time.time()
        uptime = current_time - self.startup_time

        # Check for stale topics
        stale_topics = []
        active_topics = []
        unknown_topics = []

        for topic in self.monitor_topics:
            if topic not in self.last_received or self.last_received[topic] is None:
                unknown_topics.append(topic)
                self.topic_status[topic] = 'NO_DATA'
            elif current_time - self.last_received[topic] > 2.0:  # No data for 2+ seconds
                stale_topics.append(topic)
                self.topic_status[topic] = 'STALE'
            else:
                active_topics.append(topic)
                self.topic_status[topic] = 'ACTIVE'

        # Generate status report
        timestamp = datetime.now().strftime('%H:%M:%S')

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🚁 WHEELCHAIR SYSTEM STATUS [{timestamp}]')
        self.get_logger().info(f'⏱️  System Uptime: {uptime:.1f} seconds')
        self.get_logger().info('=' * 60)

        # Active topics
        if active_topics:
            self.get_logger().info(f'✅ ACTIVE TOPICS ({len(active_topics)}):')
            for topic in active_topics:
                age = current_time - self.last_received[topic] if self.last_received[topic] else 0
                self.get_logger().info(f'   📡 {topic} (last: {age:.1f}s ago)')

        # Stale topics
        if stale_topics:
            self.get_logger().warn(f'⚠️  STALE TOPICS ({len(stale_topics)}):')
            for topic in stale_topics:
                age = current_time - self.last_received[topic] if self.last_received[topic] else float('inf')
                self.get_logger().warn(f'   🔴 {topic} (last: {age:.1f}s ago)')

        # No data topics
        if unknown_topics:
            self.get_logger().error(f'❌ NO DATA TOPICS ({len(unknown_topics)}):')
            for topic in unknown_topics:
                self.get_logger().error(f'   💀 {topic} (never received)')

        # Overall system health
        total_topics = len(self.monitor_topics)
        healthy_topics = len(active_topics)
        health_percentage = (healthy_topics / total_topics) * 100 if total_topics > 0 else 0

        if health_percentage >= 80:
            health_status = '🟢 HEALTHY'
        elif health_percentage >= 60:
            health_status = '🟡 DEGRADED'
        else:
            health_status = '🔴 CRITICAL'

        self.get_logger().info(f'🏥 SYSTEM HEALTH: {health_status} ({health_percentage:.0f}%)')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)

    try:
        monitor = SystemStatusMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in system status monitor: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()