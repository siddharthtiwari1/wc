#!/usr/bin/env python3
"""
Square Odometry Test - World-Class Analysis Tool
Tests wheelchair odometry by tracking 4x4m square path
Compares Arduino raw odom vs Filtered EKF odom with real-time plotting
Expected path: 4m forward → 90° left → 4m forward → 90° left → 4m forward → 90° left → 4m forward
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import math
import time

class SquareOdometryTest(Node):
    def __init__(self):
        super().__init__('square_odometry_test')

        # Subscriptions
        self.create_subscription(Odometry, '/wc_control/odom', self.arduino_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.filtered_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Data storage
        self.arduino_x = deque(maxlen=10000)
        self.arduino_y = deque(maxlen=10000)
        self.arduino_yaw = deque(maxlen=10000)

        self.filtered_x = deque(maxlen=10000)
        self.filtered_y = deque(maxlen=10000)
        self.filtered_yaw = deque(maxlen=10000)

        self.imu_yaw = deque(maxlen=10000)

        # Timestamps
        self.timestamps = deque(maxlen=10000)
        self.start_time = None

        # Current values
        self.current_arduino = {'x': 0, 'y': 0, 'yaw': 0}
        self.current_filtered = {'x': 0, 'y': 0, 'yaw': 0}
        self.current_imu_yaw = 0

        # Test statistics
        self.test_started = False
        self.test_start_time = None
        self.segment = 1  # Track which segment of the square we're on (1-4)

        self.get_logger().info('Square Odometry Test Node Started')
        self.get_logger().info('Expected path: 4x4m square')
        self.get_logger().info('Path: (0,0) → (4,0) → (4,4) → (0,4) → (0,0)')

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle in degrees"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)

    def arduino_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        if self.start_time is None:
            self.start_time = time.time()
            self.test_start_time = self.start_time

        t = time.time() - self.start_time

        self.arduino_x.append(x)
        self.arduino_y.append(y)
        self.arduino_yaw.append(yaw)
        self.timestamps.append(t)

        self.current_arduino = {'x': x, 'y': y, 'yaw': yaw}

        # Detect segment transitions based on position and yaw
        distance_from_start = math.sqrt(x**2 + y**2)

        if distance_from_start > 1.0:
            if x > 3.0 and abs(y) < 1.0 and abs(yaw) < 30:
                self.segment = 1
            elif x > 3.0 and y > 1.0 and 60 < yaw < 120:
                self.segment = 2
            elif abs(x) < 2.0 and y > 3.0 and 150 < abs(yaw) < 210:
                self.segment = 3
            elif abs(x) < 1.0 and y > 1.0 and (yaw < -120 or yaw > 240):
                self.segment = 4

    def filtered_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        self.filtered_x.append(x)
        self.filtered_y.append(y)
        self.filtered_yaw.append(yaw)

        self.current_filtered = {'x': x, 'y': y, 'yaw': yaw}

    def imu_callback(self, msg):
        yaw = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.imu_yaw.append(yaw)
        self.current_imu_yaw = yaw

    def calculate_metrics(self):
        """Calculate test metrics"""
        if len(self.arduino_x) < 10 or len(self.filtered_x) < 10:
            return None

        # Ideal square: (0,0) → (4,0) → (4,4) → (0,4) → (0,0)
        ideal_start = (0.0, 0.0)
        ideal_corner1 = (4.0, 0.0)
        ideal_corner2 = (4.0, 4.0)
        ideal_corner3 = (0.0, 4.0)
        ideal_end = (0.0, 0.0)

        # Current endpoints
        arduino_pos = (self.current_arduino['x'], self.current_arduino['y'])
        filtered_pos = (self.current_filtered['x'], self.current_filtered['y'])

        # Calculate distance to start (closure error)
        arduino_closure = math.sqrt(arduino_pos[0]**2 + arduino_pos[1]**2)
        filtered_closure = math.sqrt(filtered_pos[0]**2 + filtered_pos[1]**2)

        # Position difference between sources
        position_diff = math.sqrt(
            (arduino_pos[0] - filtered_pos[0])**2 +
            (arduino_pos[1] - filtered_pos[1])**2
        )

        # Orientation comparison
        yaw_diff_filtered = abs(self.current_filtered['yaw'] - self.current_imu_yaw)
        yaw_diff_arduino = abs(self.current_arduino['yaw'] - self.current_imu_yaw)

        # Normalize to 0-180 range
        if yaw_diff_filtered > 180:
            yaw_diff_filtered = 360 - yaw_diff_filtered
        if yaw_diff_arduino > 180:
            yaw_diff_arduino = 360 - yaw_diff_arduino

        # Calculate total path length traveled
        arduino_dist = 0
        filtered_dist = 0

        if len(self.arduino_x) > 1:
            for i in range(1, len(self.arduino_x)):
                dx = self.arduino_x[i] - self.arduino_x[i-1]
                dy = self.arduino_y[i] - self.arduino_y[i-1]
                arduino_dist += math.sqrt(dx**2 + dy**2)

        if len(self.filtered_x) > 1:
            for i in range(1, len(self.filtered_x)):
                dx = self.filtered_x[i] - self.filtered_x[i-1]
                dy = self.filtered_y[i] - self.filtered_y[i-1]
                filtered_dist += math.sqrt(dx**2 + dy**2)

        return {
            'arduino_pos': arduino_pos,
            'filtered_pos': filtered_pos,
            'arduino_closure': arduino_closure,
            'filtered_closure': filtered_closure,
            'position_diff': position_diff,
            'arduino_yaw': self.current_arduino['yaw'],
            'filtered_yaw': self.current_filtered['yaw'],
            'imu_yaw': self.current_imu_yaw,
            'yaw_diff_filtered': yaw_diff_filtered,
            'yaw_diff_arduino': yaw_diff_arduino,
            'arduino_dist': arduino_dist,
            'filtered_dist': filtered_dist,
            'segment': self.segment,
            'elapsed_time': time.time() - self.test_start_time if self.test_start_time else 0
        }

def main():
    rclpy.init()
    node = SquareOdometryTest()

    # Setup matplotlib
    plt.style.use('seaborn-v0_8-darkgrid')
    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

    # Main trajectory plot
    ax_traj = fig.add_subplot(gs[:2, :2])
    ax_traj.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax_traj.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    ax_traj.set_title('Square Path: 4m × 4m (16m total perimeter)',
                      fontsize=14, fontweight='bold')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_aspect('equal')

    # Plot ideal square path
    square_x = [0, 4, 4, 0, 0]
    square_y = [0, 0, 4, 4, 0]
    ax_traj.plot(square_x, square_y, 'k--', linewidth=3, alpha=0.5, label='Ideal Square Path')

    # Mark corners
    ax_traj.plot(0, 0, 'go', markersize=15, label='Start/End')
    ax_traj.plot(4, 0, 'ks', markersize=12, label='Corner 1')
    ax_traj.plot(4, 4, 'ks', markersize=12)
    ax_traj.plot(0, 4, 'ks', markersize=12)

    # Add corner labels
    ax_traj.text(0, 0, ' START', fontsize=10, fontweight='bold', va='bottom')
    ax_traj.text(4, 0, ' C1', fontsize=9, va='bottom')
    ax_traj.text(4, 4, ' C2', fontsize=9, va='bottom')
    ax_traj.text(0, 4, ' C3', fontsize=9, va='bottom')

    # Lines for actual paths
    line_arduino, = ax_traj.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Arduino Odom')
    line_filtered, = ax_traj.plot([], [], 'r-', linewidth=2, alpha=0.7, label='Filtered Odom')

    # Current position markers
    marker_arduino, = ax_traj.plot([], [], 'bo', markersize=10)
    marker_filtered, = ax_traj.plot([], [], 'ro', markersize=10)

    ax_traj.legend(loc='upper right', fontsize=10)
    ax_traj.set_xlim(-1, 5)
    ax_traj.set_ylim(-1, 5)

    # X position vs time
    ax_x = fig.add_subplot(gs[0, 2])
    ax_x.set_ylabel('X Position (m)', fontsize=10, fontweight='bold')
    ax_x.set_title('X Progress', fontsize=11, fontweight='bold')
    ax_x.grid(True, alpha=0.3)
    line_x_arduino, = ax_x.plot([], [], 'b-', linewidth=2, label='Arduino')
    line_x_filtered, = ax_x.plot([], [], 'r-', linewidth=2, label='Filtered')
    ax_x.axhline(y=4, color='k', linestyle='--', alpha=0.5, linewidth=1)
    ax_x.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1)
    ax_x.legend(loc='upper left', fontsize=8)

    # Y position vs time
    ax_y = fig.add_subplot(gs[1, 2])
    ax_y.set_ylabel('Y Position (m)', fontsize=10, fontweight='bold')
    ax_y.set_title('Y Progress', fontsize=11, fontweight='bold')
    ax_y.grid(True, alpha=0.3)
    line_y_arduino, = ax_y.plot([], [], 'b-', linewidth=2, label='Arduino')
    line_y_filtered, = ax_y.plot([], [], 'r-', linewidth=2, label='Filtered')
    ax_y.axhline(y=4, color='k', linestyle='--', alpha=0.5, linewidth=1)
    ax_y.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1)
    ax_y.legend(loc='upper left', fontsize=8)

    # Orientation vs time
    ax_yaw = fig.add_subplot(gs[2, 2])
    ax_yaw.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax_yaw.set_ylabel('Yaw (degrees)', fontsize=10, fontweight='bold')
    ax_yaw.set_title('Orientation', fontsize=11, fontweight='bold')
    ax_yaw.grid(True, alpha=0.3)
    line_yaw_imu, = ax_yaw.plot([], [], 'g-', linewidth=2.5, label='IMU', alpha=0.8)
    line_yaw_arduino, = ax_yaw.plot([], [], 'b-', linewidth=1.5, label='Arduino', alpha=0.7)
    line_yaw_filtered, = ax_yaw.plot([], [], 'r-', linewidth=1.5, label='Filtered', alpha=0.7)
    # Expected yaw angles at each corner: 0° → 90° → 180° → -90° → 0°
    for angle in [0, 90, 180, -90]:
        ax_yaw.axhline(y=angle, color='k', linestyle='--', alpha=0.3, linewidth=1)
    ax_yaw.legend(loc='upper left', fontsize=8)

    # Metrics text
    ax_metrics = fig.add_subplot(gs[2, :2])
    ax_metrics.axis('off')
    metrics_text = ax_metrics.text(0.05, 0.95, '', transform=ax_metrics.transAxes,
                                   fontsize=11, verticalalignment='top',
                                   fontfamily='monospace',
                                   bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

    def update(frame):
        rclpy.spin_once(node, timeout_sec=0.01)

        if len(node.arduino_x) < 2:
            return

        # Update trajectory
        line_arduino.set_data(list(node.arduino_x), list(node.arduino_y))
        line_filtered.set_data(list(node.filtered_x), list(node.filtered_y))

        marker_arduino.set_data([node.current_arduino['x']], [node.current_arduino['y']])
        marker_filtered.set_data([node.current_filtered['x']], [node.current_filtered['y']])

        # Auto-scale trajectory plot to keep square visible
        all_x = list(node.arduino_x) + list(node.filtered_x) + [0, 4]
        all_y = list(node.arduino_y) + list(node.filtered_y) + [0, 4]
        x_margin = 1
        y_margin = 1
        ax_traj.set_xlim(min(all_x + [-1]) - x_margin, max(all_x + [5]) + x_margin)
        ax_traj.set_ylim(min(all_y + [-1]) - y_margin, max(all_y + [5]) + y_margin)

        # Update time series - sync lengths with timestamps
        if len(node.timestamps) > 0:
            times = list(node.timestamps)

            # Get minimum length to ensure all arrays match
            min_len = min(len(times), len(node.arduino_x), len(node.arduino_y),
                         len(node.filtered_x), len(node.filtered_y),
                         len(node.arduino_yaw), len(node.filtered_yaw))

            # Truncate all arrays to the same length
            times_sync = times[-min_len:]
            arduino_x_sync = list(node.arduino_x)[-min_len:]
            arduino_y_sync = list(node.arduino_y)[-min_len:]
            filtered_x_sync = list(node.filtered_x)[-min_len:]
            filtered_y_sync = list(node.filtered_y)[-min_len:]
            arduino_yaw_sync = list(node.arduino_yaw)[-min_len:]
            filtered_yaw_sync = list(node.filtered_yaw)[-min_len:]

            line_x_arduino.set_data(times_sync, arduino_x_sync)
            line_x_filtered.set_data(times_sync, filtered_x_sync)

            line_y_arduino.set_data(times_sync, arduino_y_sync)
            line_y_filtered.set_data(times_sync, filtered_y_sync)

            # Orientation
            line_yaw_arduino.set_data(times_sync, arduino_yaw_sync)
            line_yaw_filtered.set_data(times_sync, filtered_yaw_sync)

            if len(node.imu_yaw) > 0:
                imu_len = min(len(times_sync), len(node.imu_yaw))
                line_yaw_imu.set_data(times_sync[-imu_len:], list(node.imu_yaw)[-imu_len:])

            # Auto-scale time plots
            max_time = max(times_sync) if len(times_sync) > 0 else 1
            ax_x.set_xlim(0, max_time + 1)
            ax_y.set_xlim(0, max_time + 1)
            ax_yaw.set_xlim(0, max_time + 1)

            ax_x.set_ylim(min(arduino_x_sync + filtered_x_sync + [0]) - 0.5,
                         max(arduino_x_sync + filtered_x_sync + [4]) + 0.5)
            ax_y.set_ylim(min(arduino_y_sync + filtered_y_sync + [0]) - 0.5,
                         max(arduino_y_sync + filtered_y_sync + [4]) + 0.5)

            all_yaws = arduino_yaw_sync + filtered_yaw_sync + (list(node.imu_yaw)[-imu_len:] if len(node.imu_yaw) > 0 else [])
            if len(all_yaws) > 0:
                ax_yaw.set_ylim(min(all_yaws + [-100]) - 10, max(all_yaws + [190]) + 10)

        # Update metrics
        metrics = node.calculate_metrics()
        if metrics:
            metrics_str = f"""
╔════════════════════════════════════════════════════════════════════════════════════════════════════╗
║  SQUARE PATH ODOMETRY METRICS                               Segment: {metrics['segment']}/4  Time: {metrics['elapsed_time']:6.1f}s  ║
╠════════════════════════════════════════════════════════════════════════════════════════════════════╣
║  CURRENT POSITION:                                                                                  ║
║    Arduino:   ({metrics['arduino_pos'][0]:6.3f}, {metrics['arduino_pos'][1]:6.3f})  │  Closure Error: {metrics['arduino_closure']:5.3f}m                  ║
║    Filtered:  ({metrics['filtered_pos'][0]:6.3f}, {metrics['filtered_pos'][1]:6.3f})  │  Closure Error: {metrics['filtered_closure']:5.3f}m                  ║
║    Difference between sources: {metrics['position_diff']:5.3f}m                                                    ║
║                                                                                                     ║
║  DISTANCE TRAVELED:                                                                                 ║
║    Arduino:   {metrics['arduino_dist']:6.3f}m  │  Expected: 16.000m  │  Error: {abs(metrics['arduino_dist']-16):5.3f}m                ║
║    Filtered:  {metrics['filtered_dist']:6.3f}m  │  Expected: 16.000m  │  Error: {abs(metrics['filtered_dist']-16):5.3f}m                ║
║                                                                                                     ║
║  ORIENTATION:                                                                                       ║
║    IMU:       {metrics['imu_yaw']:7.2f}°  (TRUSTED REFERENCE)                                                ║
║    Arduino:   {metrics['arduino_yaw']:7.2f}°  │  Diff from IMU: {metrics['yaw_diff_arduino']:5.2f}°                                        ║
║    Filtered:  {metrics['filtered_yaw']:7.2f}°  │  Diff from IMU: {metrics['yaw_diff_filtered']:5.2f}°                                        ║
╚════════════════════════════════════════════════════════════════════════════════════════════════════╝
            """
            metrics_text.set_text(metrics_str)

        return (line_arduino, line_filtered, marker_arduino, marker_filtered,
                line_x_arduino, line_x_filtered, line_y_arduino, line_y_filtered,
                line_yaw_imu, line_yaw_arduino, line_yaw_filtered, metrics_text)

    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
