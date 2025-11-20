#!/usr/bin/env python3
"""
Enhanced Square Odometry Test - Professional Analysis & Recording Tool
Tests wheelchair odometry with 4x4m square path tracking
Features: Real-time visualization, video recording, advanced metrics
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow, Circle, Wedge
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.collections import LineCollection
import numpy as np
from collections import deque
import math
import time
from datetime import datetime
import argparse
import sys

class SquareOdometryTestEnhanced(Node):
    def __init__(self, record_video=False, output_file=None):
        super().__init__('square_odometry_test_enhanced')

        # Recording parameters
        self.record_video = record_video
        self.output_file = output_file or f"square_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"

        # Subscriptions
        self.create_subscription(Odometry, '/wc_control/odom', self.arduino_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.filtered_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Data storage with larger buffer
        self.arduino_x = deque(maxlen=15000)
        self.arduino_y = deque(maxlen=15000)
        self.arduino_yaw = deque(maxlen=15000)

        self.filtered_x = deque(maxlen=15000)
        self.filtered_y = deque(maxlen=15000)
        self.filtered_yaw = deque(maxlen=15000)

        self.imu_yaw = deque(maxlen=15000)

        # Timestamps
        self.timestamps = deque(maxlen=15000)
        self.start_time = None

        # Current values
        self.current_arduino = {'x': 0, 'y': 0, 'yaw': 0}
        self.current_filtered = {'x': 0, 'y': 0, 'yaw': 0}
        self.current_imu_yaw = 0

        # Test statistics
        self.test_started = False
        self.test_start_time = None
        self.segment = 1
        self.corners_reached = []
        self.last_corner_time = None

        # Performance tracking
        self.max_position_error = 0
        self.avg_position_error = 0
        self.error_samples = deque(maxlen=1000)

        self.get_logger().info('╔════════════════════════════════════════════════════════╗')
        self.get_logger().info('║  Enhanced Square Odometry Test - Professional Edition  ║')
        self.get_logger().info('╠════════════════════════════════════════════════════════╣')
        self.get_logger().info('║  Path: 4×4m Square (16m perimeter)                     ║')
        self.get_logger().info('║  Route: (0,0)→(4,0)→(4,4)→(0,4)→(0,0)                  ║')
        if self.record_video:
            self.get_logger().info(f'║  Recording: {self.output_file:38} ║')
        self.get_logger().info('╚════════════════════════════════════════════════════════╝')

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

        # Track position error
        if len(self.filtered_x) > 0:
            error = math.sqrt((x - self.filtered_x[-1])**2 + (y - self.filtered_y[-1])**2)
            self.error_samples.append(error)
            if error > self.max_position_error:
                self.max_position_error = error
            if len(self.error_samples) > 0:
                self.avg_position_error = sum(self.error_samples) / len(self.error_samples)

        # Detect corners reached
        corners = [
            (4.0, 0.0, 1),
            (4.0, 4.0, 2),
            (0.0, 4.0, 3),
            (0.0, 0.0, 4)
        ]

        for cx, cy, corner_id in corners:
            if corner_id not in self.corners_reached:
                dist = math.sqrt((x - cx)**2 + (y - cy)**2)
                if dist < 0.5:  # Within 50cm of corner
                    self.corners_reached.append(corner_id)
                    self.last_corner_time = t
                    self.get_logger().info(f'✓ Corner {corner_id}/4 reached at ({x:.2f}, {y:.2f}) - Time: {t:.1f}s')

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
        """Calculate comprehensive test metrics"""
        if len(self.arduino_x) < 10 or len(self.filtered_x) < 10:
            return None

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

        # Calculate path completion percentage
        expected_dist = 16.0  # 4m × 4 sides
        completion_pct = min(100, (arduino_dist / expected_dist) * 100)

        # Determine segment based on position
        x, y = arduino_pos
        if y < 1 and x > 0:
            segment = 1  # Bottom edge
        elif x > 3 and y > 0:
            segment = 2  # Right edge
        elif y > 3 and x < 4:
            segment = 3  # Top edge
        elif x < 1 and y > 0:
            segment = 4  # Left edge
        else:
            segment = 0  # Unknown/transition

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
            'segment': segment,
            'corners_reached': len(self.corners_reached),
            'completion_pct': completion_pct,
            'max_error': self.max_position_error,
            'avg_error': self.avg_position_error,
            'elapsed_time': time.time() - self.test_start_time if self.test_start_time else 0
        }

def create_gradient_line(x, y, cmap_name='viridis', linewidth=2):
    """Create a line with gradient coloring based on progression"""
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Create LineCollection with gradient
    lc = LineCollection(segments, cmap=cmap_name, linewidth=linewidth)
    lc.set_array(np.linspace(0, 1, len(x)))
    return lc

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Enhanced Square Odometry Test')
    parser.add_argument('--record', action='store_true', help='Record video of the test')
    parser.add_argument('--output', type=str, default=None, help='Output video filename')
    parser.add_argument('--fps', type=int, default=30, help='Video FPS (default: 30)')
    parser.add_argument('--dpi', type=int, default=150, help='Video DPI quality (default: 150)')

    # Only parse known args to avoid ROS2 args conflict
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = SquareOdometryTestEnhanced(record_video=args.record, output_file=args.output)

    # Setup matplotlib with professional style - MAXIMIZED
    plt.style.use('seaborn-v0_8-darkgrid')
    fig = plt.figure(figsize=(22, 12), facecolor='#f0f0f0')

    # Try to maximize window (works with TkAgg backend)
    try:
        fig.canvas.manager.window.state('zoomed')
    except:
        try:
            fig.canvas.manager.full_screen_toggle()
        except:
            pass  # If maximizing doesn't work, just use the large figsize

    gs = fig.add_gridspec(3, 4, hspace=0.30, wspace=0.30, left=0.04, right=0.99, top=0.97, bottom=0.04)

    # === MAIN TRAJECTORY PLOT (Larger) ===
    ax_traj = fig.add_subplot(gs[:2, :3])
    ax_traj.set_xlabel('X Position (meters)', fontsize=13, fontweight='bold')
    ax_traj.set_ylabel('Y Position (meters)', fontsize=13, fontweight='bold')
    ax_traj.set_title('4×4m Square Path Tracking - Real-Time Trajectory Comparison',
                      fontsize=15, fontweight='bold', pad=15, color='#2c3e50')
    ax_traj.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
    ax_traj.set_aspect('equal')
    ax_traj.set_facecolor('#fafafa')

    # Plot ideal square path with enhanced styling
    square_x = [0, 4, 4, 0, 0]
    square_y = [0, 0, 4, 4, 0]
    ax_traj.plot(square_x, square_y, 'k--', linewidth=3.5, alpha=0.6,
                 label='Ideal Path', zorder=5)

    # Fill the square area
    ax_traj.fill(square_x, square_y, color='yellow', alpha=0.08, zorder=1)

    # Mark corners with enhanced markers
    corner_colors = ['#27ae60', '#3498db', '#9b59b6', '#e74c3c']
    corner_labels = ['Start/End', 'Corner 1', 'Corner 2', 'Corner 3']
    corners = [(0, 0), (4, 0), (4, 4), (0, 4)]

    for i, (cx, cy) in enumerate(corners):
        # Outer circle - smaller and more subtle
        circle = Circle((cx, cy), 0.12, color=corner_colors[i], alpha=0.25, zorder=4)
        ax_traj.add_patch(circle)
        # Inner marker
        ax_traj.plot(cx, cy, 'o', color=corner_colors[i], markersize=12,
                    markeredgecolor='white', markeredgewidth=2, zorder=6,
                    label=corner_labels[i] if i == 0 else f'Corner {i}')
        # NO TEXT LABELS INSIDE THE PLOT - they're in the legend instead

    # Lines for actual paths
    line_arduino, = ax_traj.plot([], [], '-', linewidth=2.5, alpha=0.85,
                                 color='#3498db', label='Arduino Odom', zorder=3)
    line_filtered, = ax_traj.plot([], [], '-', linewidth=2.5, alpha=0.85,
                                  color='#e74c3c', label='Filtered EKF', zorder=3)

    # Current position markers with direction arrows
    marker_arduino, = ax_traj.plot([], [], 'o', markersize=13, color='#3498db',
                                   markeredgecolor='white', markeredgewidth=2.5, zorder=7)
    marker_filtered, = ax_traj.plot([], [], 'o', markersize=13, color='#e74c3c',
                                    markeredgecolor='white', markeredgewidth=2.5, zorder=7)

    # Direction arrows (will be updated dynamically)
    arrow_arduino = None
    arrow_filtered = None

    ax_traj.legend(loc='upper right', fontsize=10, framealpha=0.92,
                  edgecolor='black', fancybox=True, shadow=True, ncol=2)
    ax_traj.set_xlim(-0.8, 5.2)
    ax_traj.set_ylim(-0.8, 5.2)

    # === X POSITION VS TIME ===
    ax_x = fig.add_subplot(gs[0, 3])
    ax_x.set_ylabel('X (m)', fontsize=10, fontweight='bold')
    ax_x.set_title('X Progress', fontsize=11, fontweight='bold', color='#2c3e50')
    ax_x.grid(True, alpha=0.3, linestyle='--', linewidth=0.6)
    ax_x.set_facecolor('#fafafa')
    line_x_arduino, = ax_x.plot([], [], '-', linewidth=2, color='#3498db', label='Arduino')
    line_x_filtered, = ax_x.plot([], [], '-', linewidth=2, color='#e74c3c', label='Filtered')
    ax_x.axhline(y=4, color='green', linestyle='--', alpha=0.6, linewidth=1.5, label='Target')
    ax_x.axhline(y=0, color='gray', linestyle='--', alpha=0.4, linewidth=1)
    ax_x.legend(loc='upper left', fontsize=8, framealpha=0.9)

    # === Y POSITION VS TIME ===
    ax_y = fig.add_subplot(gs[1, 3])
    ax_y.set_ylabel('Y (m)', fontsize=10, fontweight='bold')
    ax_y.set_title('Y Progress', fontsize=11, fontweight='bold', color='#2c3e50')
    ax_y.grid(True, alpha=0.3, linestyle='--', linewidth=0.6)
    ax_y.set_facecolor('#fafafa')
    line_y_arduino, = ax_y.plot([], [], '-', linewidth=2, color='#3498db', label='Arduino')
    line_y_filtered, = ax_y.plot([], [], '-', linewidth=2, color='#e74c3c', label='Filtered')
    ax_y.axhline(y=4, color='green', linestyle='--', alpha=0.6, linewidth=1.5, label='Target')
    ax_y.axhline(y=0, color='gray', linestyle='--', alpha=0.4, linewidth=1)
    ax_y.legend(loc='upper left', fontsize=8, framealpha=0.9)

    # === ORIENTATION VS TIME ===
    ax_yaw = fig.add_subplot(gs[2, 3])
    ax_yaw.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax_yaw.set_ylabel('Yaw (°)', fontsize=10, fontweight='bold')
    ax_yaw.set_title('Orientation', fontsize=11, fontweight='bold', color='#2c3e50')
    ax_yaw.grid(True, alpha=0.3, linestyle='--', linewidth=0.6)
    ax_yaw.set_facecolor('#fafafa')
    line_yaw_imu, = ax_yaw.plot([], [], '-', linewidth=2.5, color='#27ae60',
                                label='IMU', alpha=0.9)
    line_yaw_arduino, = ax_yaw.plot([], [], '-', linewidth=1.8, color='#3498db',
                                    label='Arduino', alpha=0.75)
    line_yaw_filtered, = ax_yaw.plot([], [], '-', linewidth=1.8, color='#e74c3c',
                                     label='Filtered', alpha=0.75)
    # Expected yaw angles
    for angle in [0, 90, 180, -90]:
        ax_yaw.axhline(y=angle, color='gray', linestyle='--', alpha=0.3, linewidth=1)
    ax_yaw.legend(loc='upper left', fontsize=8, framealpha=0.9)

    # === METRICS PANEL ===
    ax_metrics = fig.add_subplot(gs[2, :3])
    ax_metrics.axis('off')
    metrics_text = ax_metrics.text(0.02, 0.98, '', transform=ax_metrics.transAxes,
                                   fontsize=10, verticalalignment='top',
                                   fontfamily='monospace',
                                   bbox=dict(boxstyle='round,pad=0.8',
                                           facecolor='#ecf0f1',
                                           edgecolor='#34495e',
                                           linewidth=2,
                                           alpha=0.95))

    # Recording indicator
    if args.record:
        recording_text = fig.text(0.99, 0.99, '⏺ RECORDING',
                                 fontsize=12, fontweight='bold',
                                 color='red', ha='right', va='top',
                                 bbox=dict(boxstyle='round,pad=0.5',
                                         facecolor='white',
                                         edgecolor='red',
                                         linewidth=2))

    # Progress bar
    ax_progress = fig.add_axes([0.05, 0.01, 0.7, 0.02])
    ax_progress.set_xlim(0, 100)
    ax_progress.set_ylim(0, 1)
    ax_progress.axis('off')
    progress_bar = Rectangle((0, 0), 0, 1, facecolor='#27ae60', alpha=0.7)
    ax_progress.add_patch(progress_bar)
    progress_text = ax_progress.text(50, 0.5, '0% Complete', ha='center', va='center',
                                    fontsize=10, fontweight='bold', color='white')

    def update(frame):
        rclpy.spin_once(node, timeout_sec=0.01)

        if len(node.arduino_x) < 2:
            return []

        # Update trajectory
        arduino_x_list = list(node.arduino_x)
        arduino_y_list = list(node.arduino_y)
        filtered_x_list = list(node.filtered_x)
        filtered_y_list = list(node.filtered_y)

        line_arduino.set_data(arduino_x_list, arduino_y_list)
        line_filtered.set_data(filtered_x_list, filtered_y_list)

        marker_arduino.set_data([node.current_arduino['x']], [node.current_arduino['y']])
        marker_filtered.set_data([node.current_filtered['x']], [node.current_filtered['y']])

        # Update direction arrows
        nonlocal arrow_arduino, arrow_filtered
        if arrow_arduino:
            arrow_arduino.remove()
        if arrow_filtered:
            arrow_filtered.remove()

        # Draw direction arrows
        if len(arduino_x_list) > 5:
            # Arduino arrow
            dx_a = arduino_x_list[-1] - arduino_x_list[-5]
            dy_a = arduino_y_list[-1] - arduino_y_list[-5]
            mag_a = math.sqrt(dx_a**2 + dy_a**2)
            if mag_a > 0.01:
                arrow_arduino = FancyArrow(node.current_arduino['x'], node.current_arduino['y'],
                                         dx_a * 0.3, dy_a * 0.3,
                                         width=0.08, head_width=0.2, head_length=0.15,
                                         fc='#3498db', ec='white', linewidth=1.5,
                                         alpha=0.8, zorder=8)
                ax_traj.add_patch(arrow_arduino)

        if len(filtered_x_list) > 5:
            # Filtered arrow
            dx_f = filtered_x_list[-1] - filtered_x_list[-5]
            dy_f = filtered_y_list[-1] - filtered_y_list[-5]
            mag_f = math.sqrt(dx_f**2 + dy_f**2)
            if mag_f > 0.01:
                arrow_filtered = FancyArrow(node.current_filtered['x'], node.current_filtered['y'],
                                          dx_f * 0.3, dy_f * 0.3,
                                          width=0.08, head_width=0.2, head_length=0.15,
                                          fc='#e74c3c', ec='white', linewidth=1.5,
                                          alpha=0.8, zorder=8)
                ax_traj.add_patch(arrow_filtered)

        # Update time series
        if len(node.timestamps) > 0:
            times = list(node.timestamps)
            min_len = min(len(times), len(node.arduino_x), len(node.arduino_y),
                         len(node.filtered_x), len(node.filtered_y),
                         len(node.arduino_yaw), len(node.filtered_yaw))

            times_sync = times[-min_len:]
            arduino_x_sync = arduino_x_list[-min_len:]
            arduino_y_sync = arduino_y_list[-min_len:]
            filtered_x_sync = filtered_x_list[-min_len:]
            filtered_y_sync = filtered_y_list[-min_len:]
            arduino_yaw_sync = list(node.arduino_yaw)[-min_len:]
            filtered_yaw_sync = list(node.filtered_yaw)[-min_len:]

            line_x_arduino.set_data(times_sync, arduino_x_sync)
            line_x_filtered.set_data(times_sync, filtered_x_sync)
            line_y_arduino.set_data(times_sync, arduino_y_sync)
            line_y_filtered.set_data(times_sync, filtered_y_sync)
            line_yaw_arduino.set_data(times_sync, arduino_yaw_sync)
            line_yaw_filtered.set_data(times_sync, filtered_yaw_sync)

            if len(node.imu_yaw) > 0:
                imu_len = min(len(times_sync), len(node.imu_yaw))
                line_yaw_imu.set_data(times_sync[-imu_len:], list(node.imu_yaw)[-imu_len:])

            # Auto-scale time plots
            max_time = max(times_sync) if times_sync else 1
            ax_x.set_xlim(0, max_time + 1)
            ax_y.set_xlim(0, max_time + 1)
            ax_yaw.set_xlim(0, max_time + 1)

            if arduino_x_sync and filtered_x_sync:
                ax_x.set_ylim(min(arduino_x_sync + filtered_x_sync + [0]) - 0.5,
                             max(arduino_x_sync + filtered_x_sync + [4.5]) + 0.5)
            if arduino_y_sync and filtered_y_sync:
                ax_y.set_ylim(min(arduino_y_sync + filtered_y_sync + [0]) - 0.5,
                             max(arduino_y_sync + filtered_y_sync + [4.5]) + 0.5)

            all_yaws = arduino_yaw_sync + filtered_yaw_sync
            if len(node.imu_yaw) > 0:
                all_yaws += list(node.imu_yaw)[-imu_len:]
            if all_yaws:
                ax_yaw.set_ylim(min(all_yaws + [-100]) - 15, max(all_yaws + [190]) + 15)

        # Update metrics
        metrics = node.calculate_metrics()
        if metrics:
            # Color-coded performance indicators
            closure_status_a = '✓ EXCELLENT' if metrics['arduino_closure'] < 0.2 else \
                              '○ GOOD' if metrics['arduino_closure'] < 0.5 else '✗ POOR'
            closure_status_f = '✓ EXCELLENT' if metrics['filtered_closure'] < 0.2 else \
                              '○ GOOD' if metrics['filtered_closure'] < 0.5 else '✗ POOR'

            metrics_str = f"""
╔══════════════════════════════════════════════════════════════════════════════════════════════════════════╗
║  SQUARE PATH ODOMETRY ANALYSIS  │  Corners: {metrics['corners_reached']}/4  │  Segment: {metrics['segment']}/4  │  Time: {metrics['elapsed_time']:6.1f}s  ║
╠══════════════════════════════════════════════════════════════════════════════════════════════════════════╣
║  CURRENT POSITION:                                                                                        ║
║    Arduino:   ({metrics['arduino_pos'][0]:6.3f}, {metrics['arduino_pos'][1]:6.3f}) m  │  Closure: {metrics['arduino_closure']:5.3f}m  │  {closure_status_a:12}  ║
║    Filtered:  ({metrics['filtered_pos'][0]:6.3f}, {metrics['filtered_pos'][1]:6.3f}) m  │  Closure: {metrics['filtered_closure']:5.3f}m  │  {closure_status_f:12}  ║
║    Inter-source difference: {metrics['position_diff']:5.3f}m  │  Max error: {metrics['max_error']:5.3f}m  │  Avg: {metrics['avg_error']:5.3f}m         ║
║                                                                                                           ║
║  DISTANCE TRAVELED (Expected: 16.000m):                                                                   ║
║    Arduino:   {metrics['arduino_dist']:6.3f}m  │  Error: {abs(metrics['arduino_dist']-16):+6.3f}m  │  {metrics['arduino_dist']/16*100:5.1f}% of ideal       ║
║    Filtered:  {metrics['filtered_dist']:6.3f}m  │  Error: {abs(metrics['filtered_dist']-16):+6.3f}m  │  {metrics['filtered_dist']/16*100:5.1f}% of ideal       ║
║                                                                                                           ║
║  ORIENTATION (IMU is ground truth):                                                                       ║
║    IMU:       {metrics['imu_yaw']:7.2f}° [REFERENCE]  │  Arduino Δ: {metrics['yaw_diff_arduino']:5.2f}°  │  Filtered Δ: {metrics['yaw_diff_filtered']:5.2f}°       ║
╚══════════════════════════════════════════════════════════════════════════════════════════════════════════╝
            """
            metrics_text.set_text(metrics_str)

            # Update progress bar
            progress_bar.set_width(metrics['completion_pct'])
            progress_bar.set_facecolor('#27ae60' if metrics['completion_pct'] < 90 else '#f39c12')
            progress_text.set_text(f"{metrics['completion_pct']:.1f}% Complete")

        return [line_arduino, line_filtered, marker_arduino, marker_filtered,
                line_x_arduino, line_x_filtered, line_y_arduino, line_y_filtered,
                line_yaw_imu, line_yaw_arduino, line_yaw_filtered, metrics_text,
                progress_bar, progress_text]

    # Setup animation
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    # Setup video recording if requested
    if args.record:
        try:
            from matplotlib.animation import FFMpegWriter
            writer = FFMpegWriter(fps=args.fps, bitrate=3000,
                                 metadata={'title': 'Square Odometry Test',
                                          'artist': 'Wheelchair Navigation System',
                                          'comment': '4x4m Square Path Analysis'})

            node.get_logger().info(f'Starting video recording: {node.output_file}')
            node.get_logger().info(f'Video settings: {args.fps} FPS, {args.dpi} DPI')
            node.get_logger().info('Close the plot window to stop recording and save the video')

            # Save the animation to file
            ani.save(node.output_file, writer=writer, dpi=args.dpi)
            node.get_logger().info(f'Video saved successfully: {node.output_file}')
        except Exception as e:
            node.get_logger().error(f'Recording failed: {e}')
            node.get_logger().error('Install ffmpeg: sudo apt-get install ffmpeg')
            node.get_logger().info('Showing live visualization instead...')
            plt.show()
    else:
        plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
