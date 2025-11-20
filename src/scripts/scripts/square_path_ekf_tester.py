#!/usr/bin/env python3

"""
EKF Testing Script for 15m x 15m Square Path with Ground Truth Comparison

Test procedure:
1. Start at (0, 0) facing +X
2. Move 15m forward in +X direction
3. Turn 90° CCW (left)
4. Move 15m forward in +Y direction
5. Turn 90° CCW
6. Move 15m forward in -X direction
7. Turn 90° CCW
8. Move 15m forward in -Y direction (return to origin)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
import math
from collections import deque
import threading
import time
import csv
from datetime import datetime


class SquarePathEKFTester(Node):
    def __init__(self):
        super().__init__('square_path_ekf_tester')

        # Ground truth square path (15m x 15m)
        self.ground_truth_path = self.generate_ground_truth_square(side_length=15.0)

        # Data storage (using deque for performance)
        self.plot_maxlen = 2000
        self.raw_odom_trajectory = deque(maxlen=self.plot_maxlen)
        self.filtered_odom_trajectory = deque(maxlen=self.plot_maxlen)

        # Current positions
        self.raw_odom_current = {'x': 0, 'y': 0, 'yaw': 0, 'timestamp': 0}
        self.filtered_odom_current = {'x': 0, 'y': 0, 'yaw': 0, 'timestamp': 0}

        # Latest messages
        self.raw_odom_data = None
        self.filtered_odom_data = None
        self.imu_data = None

        # Error tracking
        self.raw_errors = deque(maxlen=self.plot_maxlen)
        self.filtered_errors = deque(maxlen=self.plot_maxlen)
        self.timestamps = deque(maxlen=self.plot_maxlen)

        # Output artifacts
        self.run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.png_file = f"square_path_ekf_plot_{self.run_timestamp}.png"
        self.csv_file_handle = None
        self.csv_writer = None
        self.setup_csv_logging()

        # Thread lock
        self.data_lock = threading.Lock()

        # ROS2 Subscribers
        self.raw_odom_sub = self.create_subscription(
            Odometry, '/wc_control/odom', self.raw_odom_callback, 10)
        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        # Setup plot
        self.setup_plot()

        # Start time
        self.start_time = time.time()

        self.get_logger().info('Square Path EKF Tester Started')
        self.get_logger().info('Test: 15m × 15m Square Path with Ground Truth')
        self.get_logger().info('Red = Raw Odometry, Blue = EKF Filtered, Green = Ground Truth')
        self.get_logger().info('Press SPACE to clear and restart test')

    def generate_ground_truth_square(self, side_length=15.0, num_points_per_side=100):
        """Generate ground truth square path coordinates"""
        path = []

        # Side 1: (0,0) → (15,0) - moving in +X
        for i in range(num_points_per_side):
            x = (i / num_points_per_side) * side_length
            y = 0.0
            path.append((x, y))

        # Side 2: (15,0) → (15,15) - moving in +Y
        for i in range(num_points_per_side):
            x = side_length
            y = (i / num_points_per_side) * side_length
            path.append((x, y))

        # Side 3: (15,15) → (0,15) - moving in -X
        for i in range(num_points_per_side):
            x = side_length - (i / num_points_per_side) * side_length
            y = side_length
            path.append((x, y))

        # Side 4: (0,15) → (0,0) - moving in -Y
        for i in range(num_points_per_side):
            x = 0.0
            y = side_length - (i / num_points_per_side) * side_length
            path.append((x, y))

        return path

    def setup_csv_logging(self):
        """Setup CSV file for logging test results"""
        self.csv_file = f"square_path_ekf_test_{self.run_timestamp}.csv"

        try:
            self.csv_file_handle = open(self.csv_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file_handle)

            # Header
            header = [
                'timestamp', 'time_elapsed',
                'raw_x', 'raw_y', 'raw_yaw', 'raw_vx', 'raw_omega',
                'filtered_x', 'filtered_y', 'filtered_yaw', 'filtered_vx', 'filtered_omega',
                'raw_error_to_gt', 'filtered_error_to_gt',
                'imu_accel_x', 'imu_gyro_z', 'imu_yaw'
            ]
            self.csv_writer.writerow(header)
            self.csv_file_handle.flush()
            self.get_logger().info(f'CSV logging to: {self.csv_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to create CSV: {e}')

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

    def find_closest_ground_truth_point(self, x, y):
        """Find closest point on ground truth path and return distance"""
        min_dist = float('inf')
        for gt_x, gt_y in self.ground_truth_path:
            dist = math.sqrt((x - gt_x)**2 + (y - gt_y)**2)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def raw_odom_callback(self, msg):
        with self.data_lock:
            self.raw_odom_data = msg
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self.raw_odom_current = {'x': x, 'y': y, 'yaw': yaw, 'timestamp': timestamp}
            self.raw_odom_trajectory.append((x, y))

            # Calculate error to ground truth
            error_to_gt = self.find_closest_ground_truth_point(x, y)
            self.raw_errors.append(error_to_gt)

            self.log_data()

    def filtered_odom_callback(self, msg):
        with self.data_lock:
            self.filtered_odom_data = msg
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self.filtered_odom_current = {'x': x, 'y': y, 'yaw': yaw, 'timestamp': timestamp}
            self.filtered_odom_trajectory.append((x, y))

            # Calculate error to ground truth
            error_to_gt = self.find_closest_ground_truth_point(x, y)
            self.filtered_errors.append(error_to_gt)

            self.timestamps.append(time.time() - self.start_time)

            self.log_data()

    def imu_callback(self, msg):
        with self.data_lock:
            self.imu_data = msg

    def log_data(self):
        """Log data to CSV"""
        if not (self.raw_odom_data and self.filtered_odom_data):
            return

        try:
            time_elapsed = time.time() - self.start_time

            raw_x = self.raw_odom_data.pose.pose.position.x
            raw_y = self.raw_odom_data.pose.pose.position.y
            raw_yaw = self.quaternion_to_yaw(self.raw_odom_data.pose.pose.orientation)
            raw_vx = self.raw_odom_data.twist.twist.linear.x
            raw_omega = self.raw_odom_data.twist.twist.angular.z

            filt_x = self.filtered_odom_data.pose.pose.position.x
            filt_y = self.filtered_odom_data.pose.pose.position.y
            filt_yaw = self.quaternion_to_yaw(self.filtered_odom_data.pose.pose.orientation)
            filt_vx = self.filtered_odom_data.twist.twist.linear.x
            filt_omega = self.filtered_odom_data.twist.twist.angular.z

            raw_error = self.find_closest_ground_truth_point(raw_x, raw_y)
            filt_error = self.find_closest_ground_truth_point(filt_x, filt_y)

            imu_accel_x = self.imu_data.linear_acceleration.x if self.imu_data else 0.0
            imu_gyro_z = self.imu_data.angular_velocity.z if self.imu_data else 0.0
            imu_yaw = self.quaternion_to_yaw(self.imu_data.orientation) if self.imu_data else 0.0

            row = [
                time.time(), time_elapsed,
                raw_x, raw_y, raw_yaw, raw_vx, raw_omega,
                filt_x, filt_y, filt_yaw, filt_vx, filt_omega,
                raw_error, filt_error,
                imu_accel_x, imu_gyro_z, imu_yaw
            ]
            self.csv_writer.writerow(row)
            self.csv_file_handle.flush()
        except Exception as e:
            self.get_logger().error(f'CSV logging error: {e}')

    def setup_plot(self):
        """Setup matplotlib plot with 3 subplots"""
        self.fig = plt.figure(figsize=(18, 6))

        # Subplot 1: Trajectory comparison
        self.ax1 = plt.subplot(1, 3, 1)
        self.ax1.set_xlim(-2, 17)
        self.ax1.set_ylim(-2, 17)
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Y Position (m)')
        self.ax1.set_title('15m × 15m Square Path Test')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')

        # Draw ground truth square
        gt_x, gt_y = zip(*self.ground_truth_path)
        self.ax1.plot(gt_x, gt_y, 'g--', linewidth=2, label='Ground Truth', alpha=0.7)

        # Draw reference square
        square = patches.Rectangle((0, 0), 15, 15, linewidth=2,
                                   edgecolor='black', facecolor='none',
                                   linestyle=':', alpha=0.5)
        self.ax1.add_patch(square)

        # Corner markers
        corners = [(0, 0), (15, 0), (15, 15), (0, 15)]
        labels = ['Start\n(0,0)', '(15,0)', '(15,15)', '(0,15)']
        for (x, y), label in zip(corners, labels):
            self.ax1.plot(x, y, 'ko', markersize=8, markerfacecolor='yellow',
                         markeredgecolor='black', markeredgewidth=2)
            self.ax1.text(x, y-0.8, label, ha='center', fontsize=9, fontweight='bold',
                         bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

        # Trajectory lines
        self.raw_line, = self.ax1.plot([], [], 'r-', linewidth=2, label='Raw Odom', alpha=0.8)
        self.filtered_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='EKF Filtered', alpha=0.8)

        # Current position markers
        self.raw_marker, = self.ax1.plot([], [], 'ro', markersize=10)
        self.filtered_marker, = self.ax1.plot([], [], 'bo', markersize=10)

        self.ax1.legend(loc='upper right', fontsize=10)

        # Subplot 2: Error over time
        self.ax2 = plt.subplot(1, 3, 2)
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Error to Ground Truth (m)')
        self.ax2.set_title('Position Error vs Time')
        self.ax2.grid(True, alpha=0.3)

        self.raw_error_line, = self.ax2.plot([], [], 'r-', linewidth=2, label='Raw Odom Error', alpha=0.7)
        self.filtered_error_line, = self.ax2.plot([], [], 'b-', linewidth=2, label='EKF Filtered Error', alpha=0.7)
        self.ax2.legend(loc='upper right', fontsize=10)

        # Subplot 3: Statistics
        self.ax3 = plt.subplot(1, 3, 3)
        self.ax3.axis('off')
        self.stats_text = self.ax3.text(0.1, 0.5, '', transform=self.ax3.transAxes,
                                       verticalalignment='center', fontfamily='monospace',
                                       fontsize=10,
                                       bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgray', alpha=0.8))

        # Keyboard event
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == ' ':
            with self.data_lock:
                self.raw_odom_trajectory.clear()
                self.filtered_odom_trajectory.clear()
                self.raw_errors.clear()
                self.filtered_errors.clear()
                self.timestamps.clear()
                self.start_time = time.time()
                self.get_logger().info('Test reset - ready for new square path')

    def update_plot(self, frame):
        """Update plot with new data"""
        with self.data_lock:
            # Update trajectory
            if len(self.raw_odom_trajectory) > 0:
                raw_x, raw_y = zip(*self.raw_odom_trajectory)
                self.raw_line.set_data(raw_x, raw_y)
                self.raw_marker.set_data([self.raw_odom_current['x']], [self.raw_odom_current['y']])

            if len(self.filtered_odom_trajectory) > 0:
                filt_x, filt_y = zip(*self.filtered_odom_trajectory)
                self.filtered_line.set_data(filt_x, filt_y)
                self.filtered_marker.set_data([self.filtered_odom_current['x']], [self.filtered_odom_current['y']])

            # Update error plot
            if len(self.timestamps) > 0 and len(self.raw_errors) > 0:
                min_len = min(len(self.timestamps), len(self.raw_errors), len(self.filtered_errors))
                self.raw_error_line.set_data(self.timestamps[:min_len], self.raw_errors[:min_len])
                self.filtered_error_line.set_data(self.timestamps[:min_len], self.filtered_errors[:min_len])

                # Auto-scale error plot
                if max(self.raw_errors[:min_len] + self.filtered_errors[:min_len]) > 0:
                    self.ax2.set_xlim(0, max(self.timestamps[:min_len]) * 1.1 if self.timestamps else 1)
                    max_error = max(self.raw_errors[:min_len] + self.filtered_errors[:min_len]) * 1.2
                    self.ax2.set_ylim(0, max_error if max_error > 0.1 else 1.0)

            # Update statistics
            self.update_statistics()

        return [self.raw_line, self.filtered_line, self.raw_marker, self.filtered_marker,
                self.raw_error_line, self.filtered_error_line, self.stats_text]

    def update_statistics(self):
        """Update statistics display"""
        stats = "TEST STATISTICS\n"
        stats += "="*40 + "\n\n"

        if len(self.raw_odom_trajectory) > 1:
            # Calculate path lengths
            raw_path_length = sum(math.sqrt((self.raw_odom_trajectory[i][0] - self.raw_odom_trajectory[i-1][0])**2 +
                                           (self.raw_odom_trajectory[i][1] - self.raw_odom_trajectory[i-1][1])**2)
                                 for i in range(1, len(self.raw_odom_trajectory)))

            filt_path_length = sum(math.sqrt((self.filtered_odom_trajectory[i][0] - self.filtered_odom_trajectory[i-1][0])**2 +
                                            (self.filtered_odom_trajectory[i][1] - self.filtered_odom_trajectory[i-1][1])**2)
                                  for i in range(1, len(self.filtered_odom_trajectory)))

            stats += f"Path Lengths:\n"
            stats += f"  Ground Truth: 60.00m\n"
            stats += f"  Raw Odom:     {raw_path_length:.2f}m\n"
            stats += f"  EKF Filtered: {filt_path_length:.2f}m\n\n"

            # Error statistics
            if len(self.raw_errors) > 0:
                raw_mean_error = np.mean(self.raw_errors)
                raw_max_error = np.max(self.raw_errors)
                raw_final_error = math.sqrt(self.raw_odom_current['x']**2 + self.raw_odom_current['y']**2)

                filt_mean_error = np.mean(self.filtered_errors)
                filt_max_error = np.max(self.filtered_errors)
                filt_final_error = math.sqrt(self.filtered_odom_current['x']**2 + self.filtered_odom_current['y']**2)

                stats += f"Raw Odom Errors:\n"
                stats += f"  Mean Error:  {raw_mean_error:.3f}m\n"
                stats += f"  Max Error:   {raw_max_error:.3f}m\n"
                stats += f"  Final Error: {raw_final_error:.3f}m\n\n"

                stats += f"EKF Filtered Errors:\n"
                stats += f"  Mean Error:  {filt_mean_error:.3f}m\n"
                stats += f"  Max Error:   {filt_max_error:.3f}m\n"
                stats += f"  Final Error: {filt_final_error:.3f}m\n\n"

                improvement = ((raw_mean_error - filt_mean_error) / raw_mean_error * 100) if raw_mean_error > 0 else 0
                stats += f"EKF Improvement: {improvement:.1f}%\n\n"

            # Current position
            stats += f"Current Position:\n"
            stats += f"  Raw:  ({self.raw_odom_current['x']:.2f}, {self.raw_odom_current['y']:.2f})\n"
            stats += f"  Filt: ({self.filtered_odom_current['x']:.2f}, {self.filtered_odom_current['y']:.2f})\n\n"

            # Test progress
            test_time = time.time() - self.start_time
            stats += f"Test Duration: {test_time:.1f}s\n"
            stats += f"Data Points: {len(self.raw_odom_trajectory)}\n"
        else:
            stats += "Waiting for data...\n\n"
            stats += "Expected path:\n"
            stats += "  15m → (turn 90° CCW)\n"
            stats += "  15m → (turn 90° CCW)\n"
            stats += "  15m → (turn 90° CCW)\n"
            stats += "  15m → (return to start)\n"
            stats += "\nTotal: 60m square path"

        self.stats_text.set_text(stats)

    def start_visualization(self):
        """Start visualization"""
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
        plt.tight_layout()
        plt.ion()
        plt.show(block=False)

        try:
            while plt.fignum_exists(self.fig.number):
                plt.pause(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            plt.ioff()

    def cleanup(self):
        """Cleanup resources"""
        try:
            self.save_plot_snapshot()
            if self.csv_file_handle:
                self.csv_file_handle.close()
            self.get_logger().info(f'Test data saved to: {self.csv_file}')
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')

    def save_plot_snapshot(self):
        """Persist final plot as PNG for later review"""
        try:
            if self.fig:
                if self.fig.canvas is None:
                    FigureCanvasAgg(self.fig)
                self.fig.canvas.draw()
                self.fig.savefig(self.png_file, dpi=200, bbox_inches='tight')
                self.get_logger().info(f'Plot snapshot saved to: {self.png_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save plot snapshot: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = SquarePathEKFTester()

    # Start ROS2 in separate thread
    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            node.get_logger().info('External shutdown signal received; spin thread exiting')
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f'Spin thread error: {exc}')

    ros_thread = threading.Thread(target=spin_node, daemon=True)
    ros_thread.start()

    try:
        node.start_visualization()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RCLError as exc:
            if 'rcl_shutdown already called' not in str(exc):
                raise


if __name__ == '__main__':
    main()
