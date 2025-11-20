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

class EKFComparisonPlotter(Node):
    def __init__(self):
        super().__init__('ekf_comparison_plotter')

        # ---- Parameters (so you can override with --ros-args -p ...) ----
        self.declare_parameter('encoder_odom_topic', '/wc_control/odom')
        self.declare_parameter('ekf_odom_topic', '/odometry/filtered')
        self.declare_parameter('arrow_interval', 10)

        self.enc_topic = self.get_parameter('encoder_odom_topic').get_parameter_value().string_value
        self.ekf_topic = self.get_parameter('ekf_odom_topic').get_parameter_value().string_value
        self.arrow_interval = self.get_parameter('arrow_interval').get_parameter_value().integer_value

        # ---- Data stores ----
        self.enc_data = {'x': [], 'y': [], 'yaw': [], 'vx': [], 'vy': [], 'timestamps': []}
        self.ekf_data = {'x': [], 'y': [], 'yaw': [], 'vx': [], 'vy': [], 'timestamps': []}
        self.data_lock = Lock()

        # ---- Subscribers ----
        self.enc_sub = self.create_subscription(Odometry, self.enc_topic, self.enc_cb, 10)
        self.ekf_sub = self.create_subscription(Odometry, self.ekf_topic, self.ekf_cb, 10)

        # ---- Plot setup ----
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Main trajectory plot
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Y Position (m)')
        self.ax1.set_title(f'Trajectory Comparison: {self.enc_topic} vs {self.ekf_topic}')
        self.ax1.grid(True)
        self.ax1.axis('equal')
        
        # Error plot
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Position Error (m)')
        self.ax2.set_title('Position Error Over Time')
        self.ax2.grid(True)

        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info(f"Subscribing to {self.enc_topic} (encoder) and {self.ekf_topic} (EKF filtered)")

    def _append(self, store, msg):
        store['x'].append(msg.pose.pose.position.x)
        store['y'].append(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        store['yaw'].append(yaw)
        store['vx'].append(msg.twist.twist.linear.x)
        store['vy'].append(msg.twist.twist.linear.y)
        # Prefer message stamp (more accurate than node clock)
        ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        store['timestamps'].append(ts)

    def enc_cb(self, msg: Odometry):
        with self.data_lock:
            self._append(self.enc_data, msg)

    def ekf_cb(self, msg: Odometry):
        with self.data_lock:
            self._append(self.ekf_data, msg)

    def update_plot(self):
        with self.data_lock:
            self.ax1.clear()
            self.ax2.clear()

            # Trajectory plot
            # encoder odom path + twist arrows
            if len(self.enc_data['x']) > 1:
                self.ax1.plot(self.enc_data['x'], self.enc_data['y'],
                             'r-', label=f'{self.enc_topic} (raw)', linewidth=2, alpha=0.8)
                for i in range(0, len(self.enc_data['x']), self.arrow_interval):
                    x, y = self.enc_data['x'][i], self.enc_data['y'][i]
                    vx, vy = self.enc_data['vx'][i], self.enc_data['vy'][i]
                    arrow_scale = 0.2 / max(0.1, math.sqrt(vx*vx + vy*vy))
                    dx, dy = arrow_scale * vx, arrow_scale * vy
                    self.ax1.arrow(x, y, dx, dy, head_width=0.05, head_length=0.05,
                                  fc='red', ec='red', alpha=0.7)

            # EKF filtered path + twist arrows
            if len(self.ekf_data['x']) > 1:
                self.ax1.plot(self.ekf_data['x'], self.ekf_data['y'],
                             'b-', label=f'{self.ekf_topic} (fused)', linewidth=2, alpha=0.8)
                for i in range(0, len(self.ekf_data['x']), self.arrow_interval):
                    x, y = self.ekf_data['x'][i], self.ekf_data['y'][i]
                    vx, vy = self.ekf_data['vx'][i], self.ekf_data['vy'][i]
                    arrow_scale = 0.2 / max(0.1, math.sqrt(vx*vx + vy*vy))
                    dx, dy = arrow_scale * vx, arrow_scale * vy
                    self.ax1.arrow(x, y, dx, dy, head_width=0.05, head_length=0.05,
                                  fc='blue', ec='blue', alpha=0.7)

            # current points
            if len(self.enc_data['x']) > 0:
                self.ax1.plot(self.enc_data['x'][-1], self.enc_data['y'][-1],
                             'ro', markersize=10, label=f'{self.enc_topic} current')
            if len(self.ekf_data['x']) > 0:
                self.ax1.plot(self.ekf_data['x'][-1], self.ekf_data['y'][-1],
                             'bo', markersize=10, label=f'{self.ekf_topic} current')

            self.ax1.set_xlabel('X Position (m)')
            self.ax1.set_ylabel('Y Position (m)')
            self.ax1.set_title(f'Trajectory Comparison: Encoder vs EKF Filtered')
            self.ax1.grid(True)
            self.ax1.legend()
            self.ax1.axis('equal')

            # Error analysis plot
            if len(self.enc_data['x']) > 0 and len(self.ekf_data['x']) > 0:
                # Find common time range for comparison
                min_len = min(len(self.enc_data['x']), len(self.ekf_data['x']))
                if min_len > 1:
                    # Calculate position errors
                    position_errors = []
                    time_errors = []
                    start_time = min(self.enc_data['timestamps'][0], self.ekf_data['timestamps'][0])
                    
                    for i in range(min_len):
                        if i < len(self.enc_data['x']) and i < len(self.ekf_data['x']):
                            dx = self.enc_data['x'][i] - self.ekf_data['x'][i]
                            dy = self.enc_data['y'][i] - self.ekf_data['y'][i]
                            error = math.sqrt(dx*dx + dy*dy)
                            position_errors.append(error)
                            
                            # Use encoder timestamps as reference
                            time_errors.append(self.enc_data['timestamps'][i] - start_time)
                    
                    if len(position_errors) > 0:
                        self.ax2.plot(time_errors, position_errors, 'g-', linewidth=2, alpha=0.8)
                        self.ax2.set_xlabel('Time (s)')
                        self.ax2.set_ylabel('Position Error (m)')
                        self.ax2.set_title('Position Error: |Encoder - EKF Filtered|')
                        self.ax2.grid(True)
                        
                        # Add statistics
                        mean_error = np.mean(position_errors)
                        max_error = np.max(position_errors)
                        self.ax2.axhline(y=mean_error, color='orange', linestyle='--', 
                                        label=f'Mean: {mean_error:.3f}m')
                        self.ax2.legend()
                        
                        # Log statistics
                        if len(position_errors) % 50 == 0:  # Log every 50 samples
                            self.get_logger().info(f"Position Error Stats - Mean: {mean_error:.3f}m, Max: {max_error:.3f}m")

        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)

    def signal_handler(self, sig, frame):
        print('\nShutting down plotter...')
        plt.ioff()
        plt.show()
        rclpy.shutdown()
        sys.exit(0)

    def run(self):
        # update plot at 10 Hz
        _ = self.create_timer(0.1, self.update_plot)
        print("Starting EKF comparison plotter…")
        print(f"Subscribing to {self.enc_topic} and {self.ekf_topic}")
        print("Left plot: Trajectory comparison")
        print("Right plot: Position error over time")
        print("Press Ctrl+C to stop")
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

def main():
    rclpy.init()
    try:
        node = EKFComparisonPlotter()
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()