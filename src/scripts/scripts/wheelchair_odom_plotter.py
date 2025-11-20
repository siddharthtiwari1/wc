#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from threading import Lock
import signal
import sys
import math
from tf_transformations import euler_from_quaternion

class WheelchairOdomPlotter(Node):
    def __init__(self):
        super().__init__('wheelchair_odom_plotter')

        # ---- Parameters ----
        self.declare_parameter('raw_odom_topic', '/wc_control/odom')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('arrow_interval', 15)

        self.raw_topic = self.get_parameter('raw_odom_topic').get_parameter_value().string_value
        self.filtered_topic = self.get_parameter('filtered_odom_topic').get_parameter_value().string_value
        self.arrow_interval = self.get_parameter('arrow_interval').get_parameter_value().integer_value

        # ---- Data stores ----
        self.raw_data = {'x': [], 'y': [], 'yaw': [], 'vx': [], 'vy': [], 'timestamps': []}
        self.filtered_data = {'x': [], 'y': [], 'yaw': [], 'vx': [], 'vy': [], 'timestamps': []}
        self.data_lock = Lock()

        # ---- Subscribers ----
        self.raw_sub = self.create_subscription(Odometry, self.raw_topic, self.raw_cb, 10)
        self.filtered_sub = self.create_subscription(Odometry, self.filtered_topic, self.filtered_cb, 10)

        # ---- Plot setup ----
        plt.ion()
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # Trajectory plot
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Y Position (m)')
        self.ax1.set_title('Wheelchair Trajectory: Raw vs EKF Filtered')
        self.ax1.grid(True)
        self.ax1.axis('equal')
        
        # Position error plot
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Position Error (m)')
        self.ax2.set_title('Position Error: |Raw - Filtered|')
        self.ax2.grid(True)

        # Velocity comparison
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Linear Velocity (m/s)')
        self.ax3.set_title('Linear Velocity Comparison')
        self.ax3.grid(True)

        # Yaw comparison
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Yaw (rad)')
        self.ax4.set_title('Yaw Angle Comparison')
        self.ax4.grid(True)

        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info(f"Wheelchair Odometry Plotter")
        self.get_logger().info(f"Raw: {self.raw_topic}")
        self.get_logger().info(f"Filtered: {self.filtered_topic}")

    def _append(self, store, msg):
        store['x'].append(msg.pose.pose.position.x)
        store['y'].append(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        store['yaw'].append(yaw)
        store['vx'].append(msg.twist.twist.linear.x)
        store['vy'].append(msg.twist.twist.linear.y)
        # Use message timestamp
        ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        store['timestamps'].append(ts)

    def raw_cb(self, msg: Odometry):
        with self.data_lock:
            self._append(self.raw_data, msg)

    def filtered_cb(self, msg: Odometry):
        with self.data_lock:
            self._append(self.filtered_data, msg)

    def update_plot(self):
        with self.data_lock:
            # Clear all plots
            for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
                ax.clear()

            # Plot 1: Trajectory comparison
            if len(self.raw_data['x']) > 1:
                self.ax1.plot(self.raw_data['x'], self.raw_data['y'],
                             'r-', label='Raw Odometry', linewidth=2, alpha=0.8)
                # Add velocity arrows for raw data
                for i in range(0, len(self.raw_data['x']), self.arrow_interval):
                    x, y = self.raw_data['x'][i], self.raw_data['y'][i]
                    vx, vy = self.raw_data['vx'][i], self.raw_data['vy'][i]
                    v_mag = math.sqrt(vx*vx + vy*vy)
                    if v_mag > 0.01:  # Only show arrows for significant motion
                        arrow_scale = 0.15 / max(0.1, v_mag)
                        dx, dy = arrow_scale * vx, arrow_scale * vy
                        self.ax1.arrow(x, y, dx, dy, head_width=0.03, head_length=0.03,
                                      fc='red', ec='red', alpha=0.6)

            if len(self.filtered_data['x']) > 1:
                self.ax1.plot(self.filtered_data['x'], self.filtered_data['y'],
                             'b-', label='EKF Filtered', linewidth=2, alpha=0.8)
                # Add velocity arrows for filtered data
                for i in range(0, len(self.filtered_data['x']), self.arrow_interval):
                    x, y = self.filtered_data['x'][i], self.filtered_data['y'][i]
                    vx, vy = self.filtered_data['vx'][i], self.filtered_data['vy'][i]
                    v_mag = math.sqrt(vx*vx + vy*vy)
                    if v_mag > 0.01:  # Only show arrows for significant motion
                        arrow_scale = 0.15 / max(0.1, v_mag)
                        dx, dy = arrow_scale * vx, arrow_scale * vy
                        self.ax1.arrow(x, y, dx, dy, head_width=0.03, head_length=0.03,
                                      fc='blue', ec='blue', alpha=0.6)

            # Current position markers
            if len(self.raw_data['x']) > 0:
                self.ax1.plot(self.raw_data['x'][-1], self.raw_data['y'][-1],
                             'ro', markersize=8, label='Raw Current')
            if len(self.filtered_data['x']) > 0:
                self.ax1.plot(self.filtered_data['x'][-1], self.filtered_data['y'][-1],
                             'bo', markersize=8, label='Filtered Current')

            self.ax1.set_xlabel('X Position (m)')
            self.ax1.set_ylabel('Y Position (m)')
            self.ax1.set_title('Wheelchair Trajectory: Raw vs EKF Filtered')
            self.ax1.grid(True)
            self.ax1.legend()
            self.ax1.axis('equal')

            # Plot 2: Position error analysis
            if len(self.raw_data['x']) > 0 and len(self.filtered_data['x']) > 0:
                min_len = min(len(self.raw_data['x']), len(self.filtered_data['x']))
                if min_len > 1:
                    position_errors = []
                    time_errors = []
                    start_time = min(self.raw_data['timestamps'][0], self.filtered_data['timestamps'][0])
                    
                    for i in range(min_len):
                        if i < len(self.raw_data['x']) and i < len(self.filtered_data['x']):
                            dx = self.raw_data['x'][i] - self.filtered_data['x'][i]
                            dy = self.raw_data['y'][i] - self.filtered_data['y'][i]
                            error = math.sqrt(dx*dx + dy*dy)
                            position_errors.append(error)
                            time_errors.append(self.raw_data['timestamps'][i] - start_time)
                    
                    if len(position_errors) > 0:
                        self.ax2.plot(time_errors, position_errors, 'g-', linewidth=2)
                        mean_error = np.mean(position_errors)
                        max_error = np.max(position_errors)
                        self.ax2.axhline(y=mean_error, color='orange', linestyle='--', 
                                        label=f'Mean: {mean_error:.3f}m')
                        self.ax2.set_xlabel('Time (s)')
                        self.ax2.set_ylabel('Position Error (m)')
                        self.ax2.set_title('Position Error: |Raw - Filtered|')
                        self.ax2.grid(True)
                        self.ax2.legend()

            # Plot 3: Velocity comparison
            if len(self.raw_data['timestamps']) > 0:
                start_time = self.raw_data['timestamps'][0]
                raw_times = [t - start_time for t in self.raw_data['timestamps']]
                raw_velocities = [math.sqrt(vx*vx + vy*vy) for vx, vy in zip(self.raw_data['vx'], self.raw_data['vy'])]
                self.ax3.plot(raw_times, raw_velocities, 'r-', label='Raw Velocity', linewidth=2, alpha=0.8)

            if len(self.filtered_data['timestamps']) > 0:
                start_time = self.filtered_data['timestamps'][0] if len(self.raw_data['timestamps']) == 0 else self.raw_data['timestamps'][0]
                filtered_times = [t - start_time for t in self.filtered_data['timestamps']]
                filtered_velocities = [math.sqrt(vx*vx + vy*vy) for vx, vy in zip(self.filtered_data['vx'], self.filtered_data['vy'])]
                self.ax3.plot(filtered_times, filtered_velocities, 'b-', label='Filtered Velocity', linewidth=2, alpha=0.8)

            self.ax3.set_xlabel('Time (s)')
            self.ax3.set_ylabel('Linear Velocity (m/s)')
            self.ax3.set_title('Linear Velocity Comparison')
            self.ax3.grid(True)
            self.ax3.legend()

            # Plot 4: Yaw comparison
            if len(self.raw_data['timestamps']) > 0:
                start_time = self.raw_data['timestamps'][0]
                raw_times = [t - start_time for t in self.raw_data['timestamps']]
                self.ax4.plot(raw_times, self.raw_data['yaw'], 'r-', label='Raw Yaw', linewidth=2, alpha=0.8)

            if len(self.filtered_data['timestamps']) > 0:
                start_time = self.filtered_data['timestamps'][0] if len(self.raw_data['timestamps']) == 0 else self.raw_data['timestamps'][0]
                filtered_times = [t - start_time for t in self.filtered_data['timestamps']]
                self.ax4.plot(filtered_times, self.filtered_data['yaw'], 'b-', label='Filtered Yaw', linewidth=2, alpha=0.8)

            self.ax4.set_xlabel('Time (s)')
            self.ax4.set_ylabel('Yaw (rad)')
            self.ax4.set_title('Yaw Angle Comparison')
            self.ax4.grid(True)
            self.ax4.legend()

        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)

    def signal_handler(self, sig, frame):
        print('\nShutting down wheelchair odometry plotter...')
        plt.ioff()
        plt.show()
        rclpy.shutdown()
        sys.exit(0)

    def run(self):
        # Update plot at 10 Hz
        _ = self.create_timer(0.1, self.update_plot)
        print("Starting Wheelchair Odometry Plotter...")
        print(f"Raw odometry: {self.raw_topic}")
        print(f"Filtered odometry: {self.filtered_topic}")
        print("4-panel display:")
        print("  Top-left: Trajectory comparison with velocity arrows")
        print("  Top-right: Position error over time")
        print("  Bottom-left: Linear velocity comparison")
        print("  Bottom-right: Yaw angle comparison")
        print("Press Ctrl+C to stop")
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

def main():
    rclpy.init()
    try:
        node = WheelchairOdomPlotter()
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()