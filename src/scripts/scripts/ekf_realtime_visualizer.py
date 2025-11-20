#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
import math
from collections import deque
import threading
import time
import csv
import os
from datetime import datetime

class EKFRealtimeVisualizer(Node):
    def __init__(self):
        super().__init__('ekf_realtime_visualizer')
        
        # Data storage for trajectories (keep last 10000 points for complete rectangle paths)
        self.max_points = 10000  # Increased to prevent trajectory disappearing
        self.raw_odom_trajectory = deque(maxlen=self.max_points)
        self.filtered_odom_trajectory = deque(maxlen=self.max_points)
        
        # Store all trajectory points without deque limits to prevent disappearing
        self.raw_odom_full_trajectory = []
        self.filtered_odom_full_trajectory = []
        
        # Current positions and orientations
        self.raw_odom_current = {'x': 0, 'y': 0, 'yaw': 0, 'timestamp': 0}
        self.filtered_odom_current = {'x': 0, 'y': 0, 'yaw': 0, 'timestamp': 0}
        
        # Data for terminal display and CSV logging
        self.raw_odom_data = None
        self.filtered_odom_data = None
        self.imu_data = None
        self.imu_out_data = None
        self.imu_republisher_data = None
        
        # CSV logging setup
        self.setup_csv_logging()
        
        # Thread locks
        self.data_lock = threading.Lock()
        
        # ROS2 Subscribers
        self.raw_odom_sub = self.create_subscription(
            Odometry, '/wc_control/odom', self.raw_odom_callback, 10)
        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/odom_oriented', self.imu_callback, 10)
        
        # Additional subscribers for CSV logging
        self.imu_out_sub = self.create_subscription(
            Imu, '/imu/out', self.imu_out_callback, 10)
        self.imu_republisher_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_republisher_callback, 10)  # Common republisher topic
        
        # Setup matplotlib
        self.setup_plot()
        
        # Start terminal data display thread
        self.terminal_thread = threading.Thread(target=self.terminal_display_loop, daemon=True)
        self.terminal_thread.start()
        
        self.get_logger().info('EKF Real-time Visualizer started')
        self.get_logger().info('Rectangle area: 5m x 10m')
        self.get_logger().info('Red = Raw Odometry, Blue = Filtered Odometry (EKF)')
        self.get_logger().info('Press SPACE in plot window to clear trajectories and restart')
        self.get_logger().info(f'CSV files: {self.csv_odom_file}, {self.csv_imu_file}')

    def setup_csv_logging(self):
        """Setup CSV files for data logging"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # CSV File 1: Raw Odom and Filtered Odom
        self.csv_odom_file = f"odom_comparison_{timestamp}.csv"
        self.csv_odom_writer = None
        self.csv_odom_file_handle = None
        
        # CSV File 2: IMU Out, IMU Republisher, and Filtered Odom
        self.csv_imu_file = f"imu_ekf_comparison_{timestamp}.csv"
        self.csv_imu_writer = None
        self.csv_imu_file_handle = None
        
        # Initialize CSV files
        self.init_odom_csv()
        self.init_imu_csv()
    
    def init_odom_csv(self):
        """Initialize the odometry comparison CSV file"""
        try:
            # Ensure the file is created in current directory
            self.csv_odom_file_handle = open(self.csv_odom_file, 'w', newline='')
            self.csv_odom_writer = csv.writer(self.csv_odom_file_handle)
            
            # Write header
            header = [
                'timestamp', 'raw_odom_x', 'raw_odom_y', 'raw_odom_yaw', 'raw_odom_vel_x', 'raw_odom_vel_y', 'raw_odom_vel_angular',
                'filtered_odom_x', 'filtered_odom_y', 'filtered_odom_yaw', 'filtered_odom_vel_x', 'filtered_odom_vel_y', 'filtered_odom_vel_angular'
            ]
            self.csv_odom_writer.writerow(header)
            self.csv_odom_file_handle.flush()
            
            # Verify file was created
            if os.path.exists(self.csv_odom_file):
                self.get_logger().info(f"✅ CSV file created successfully: {self.csv_odom_file}")
            else:
                self.get_logger().error(f"❌ CSV file creation failed: {self.csv_odom_file}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to create odom CSV file: {e}")
    
    def init_imu_csv(self):
        """Initialize the IMU + EKF comparison CSV file"""
        try:
            self.csv_imu_file_handle = open(self.csv_imu_file, 'w', newline='')
            self.csv_imu_writer = csv.writer(self.csv_imu_file_handle)
            
            # Write header
            header = [
                'timestamp', 'imu_out_accel_x', 'imu_out_accel_y', 'imu_out_accel_z',
                'imu_out_gyro_x', 'imu_out_gyro_y', 'imu_out_gyro_z',
                'imu_out_orient_x', 'imu_out_orient_y', 'imu_out_orient_z', 'imu_out_orient_w',
                'imu_republisher_accel_x', 'imu_republisher_accel_y', 'imu_republisher_accel_z',
                'imu_republisher_gyro_x', 'imu_republisher_gyro_y', 'imu_republisher_gyro_z',
                'imu_republisher_orient_x', 'imu_republisher_orient_y', 'imu_republisher_orient_z', 'imu_republisher_orient_w',
                'filtered_odom_x', 'filtered_odom_y', 'filtered_odom_yaw'
            ]
            self.csv_imu_writer.writerow(header)
            self.csv_imu_file_handle.flush()
            
            # Verify file was created
            if os.path.exists(self.csv_imu_file):
                self.get_logger().info(f"✅ IMU CSV file created successfully: {self.csv_imu_file}")
            else:
                self.get_logger().error(f"❌ IMU CSV file creation failed: {self.csv_imu_file}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to create IMU CSV file: {e}")
    
    def log_odom_data(self):
        """Log odometry comparison data to CSV"""
        if self.csv_odom_writer and self.raw_odom_data and self.filtered_odom_data:
            try:
                current_time = time.time()
                
                # Raw odom data
                raw_x = self.raw_odom_data.pose.pose.position.x
                raw_y = self.raw_odom_data.pose.pose.position.y
                raw_yaw = self.quaternion_to_yaw(self.raw_odom_data.pose.pose.orientation)
                raw_vel_x = self.raw_odom_data.twist.twist.linear.x
                raw_vel_y = self.raw_odom_data.twist.twist.linear.y
                raw_vel_angular = self.raw_odom_data.twist.twist.angular.z
                
                # Filtered odom data
                filt_x = self.filtered_odom_data.pose.pose.position.x
                filt_y = self.filtered_odom_data.pose.pose.position.y
                filt_yaw = self.quaternion_to_yaw(self.filtered_odom_data.pose.pose.orientation)
                filt_vel_x = self.filtered_odom_data.twist.twist.linear.x
                filt_vel_y = self.filtered_odom_data.twist.twist.linear.y
                filt_vel_angular = self.filtered_odom_data.twist.twist.angular.z
                
                # Write row
                row = [
                    current_time, raw_x, raw_y, raw_yaw, raw_vel_x, raw_vel_y, raw_vel_angular,
                    filt_x, filt_y, filt_yaw, filt_vel_x, filt_vel_y, filt_vel_angular
                ]
                self.csv_odom_writer.writerow(row)
                self.csv_odom_file_handle.flush()
            except Exception as e:
                self.get_logger().error(f"Failed to log odom data: {e}")

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle in radians"""
        return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

    def raw_odom_callback(self, msg):
        with self.data_lock:
            self.raw_odom_data = msg
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            self.raw_odom_current = {'x': x, 'y': y, 'yaw': yaw, 'timestamp': timestamp}
            self.raw_odom_trajectory.append((x, y))
            self.raw_odom_full_trajectory.append((x, y))  # Store without limits
            
            # Log to CSV when both raw and filtered data are available
            self.log_odom_data()

    def filtered_odom_callback(self, msg):
        with self.data_lock:
            self.filtered_odom_data = msg
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            self.filtered_odom_current = {'x': x, 'y': y, 'yaw': yaw, 'timestamp': timestamp}
            self.filtered_odom_trajectory.append((x, y))
            self.filtered_odom_full_trajectory.append((x, y))  # Store without limits
            
            # Log to CSV and IMU comparison
            self.log_odom_data()
            self.log_imu_data()

    def imu_callback(self, msg):
        with self.data_lock:
            # Only store IMU data for timestamp checking and terminal display
            self.imu_data = msg
    
    def imu_out_callback(self, msg):
        with self.data_lock:
            self.imu_out_data = msg
            self.log_imu_data()
    
    def imu_republisher_callback(self, msg):
        with self.data_lock:
            self.imu_republisher_data = msg
            self.log_imu_data()
    
    def log_imu_data(self):
        """Log IMU + EKF comparison data to CSV"""
        if (self.csv_imu_writer and self.imu_out_data and 
            self.imu_republisher_data and self.filtered_odom_data):
            try:
                current_time = time.time()
                
                # IMU out data
                imu_out_accel = [self.imu_out_data.linear_acceleration.x,
                               self.imu_out_data.linear_acceleration.y,
                               self.imu_out_data.linear_acceleration.z]
                imu_out_gyro = [self.imu_out_data.angular_velocity.x,
                              self.imu_out_data.angular_velocity.y,
                              self.imu_out_data.angular_velocity.z]
                imu_out_orient = [self.imu_out_data.orientation.x,
                                self.imu_out_data.orientation.y,
                                self.imu_out_data.orientation.z,
                                self.imu_out_data.orientation.w]
                
                # IMU republisher data
                imu_repub_accel = [self.imu_republisher_data.linear_acceleration.x,
                                 self.imu_republisher_data.linear_acceleration.y,
                                 self.imu_republisher_data.linear_acceleration.z]
                imu_repub_gyro = [self.imu_republisher_data.angular_velocity.x,
                                self.imu_republisher_data.angular_velocity.y,
                                self.imu_republisher_data.angular_velocity.z]
                imu_repub_orient = [self.imu_republisher_data.orientation.x,
                                  self.imu_republisher_data.orientation.y,
                                  self.imu_republisher_data.orientation.z,
                                  self.imu_republisher_data.orientation.w]
                
                # Filtered odom data
                filt_x = self.filtered_odom_data.pose.pose.position.x
                filt_y = self.filtered_odom_data.pose.pose.position.y
                filt_yaw = self.quaternion_to_yaw(self.filtered_odom_data.pose.pose.orientation)
                
                # Write row
                row = [current_time] + imu_out_accel + imu_out_gyro + imu_out_orient + \
                      imu_repub_accel + imu_repub_gyro + imu_repub_orient + \
                      [filt_x, filt_y, filt_yaw]
                
                self.csv_imu_writer.writerow(row)
                self.csv_imu_file_handle.flush()
            except Exception as e:
                self.get_logger().error(f"Failed to log IMU data: {e}")

    def setup_plot(self):
        """Setup the matplotlib plot"""
        self.fig, self.ax = plt.subplots(figsize=(8, 12))  # Adjusted for 5x10 rectangle
        self.ax.set_xlim(-1, 6)   # 5m + 1m margin
        self.ax.set_ylim(-1, 11)  # 10m + 1m margin
        self.ax.set_xlabel('X Position (meters)')
        self.ax.set_ylabel('Y Position (meters)')
        self.ax.set_title('EKF Test: Wheelchair Trajectory Comparison (5m × 10m)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Draw 5x10 meter rectangle
        rectangle = patches.Rectangle((0, 0), 5, 10, linewidth=2, 
                                    edgecolor='black', facecolor='none', 
                                    linestyle='--', alpha=0.7)
        self.ax.add_patch(rectangle)
        
        # Add rectangle label and corner markers
        self.ax.text(2.5, 10.2, '5m × 10m Test Area', ha='center', fontsize=12, fontweight='bold')
        
        # Add corner markers for reference
        corners = [(0, 0), (5, 0), (5, 10), (0, 10)]
        corner_labels = ['Start (0,0)', '(5,0)', '(5,10)', '(0,10)']
        for i, ((x, y), label) in enumerate(zip(corners, corner_labels)):
            self.ax.plot(x, y, 'ko', markersize=8, markerfacecolor='yellow', markeredgecolor='black', markeredgewidth=2)
            offset_x = 0.2 if i % 2 == 0 else -0.2
            offset_y = 0.2 if i < 2 else -0.3
            self.ax.annotate(label, (x, y), xytext=(offset_x, offset_y), textcoords='offset points', 
                           fontsize=10, fontweight='bold', ha='center', va='center',
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
        
        # Initialize empty plots
        self.raw_line, = self.ax.plot([], [], 'r-', linewidth=3, label='Raw Odometry', alpha=0.9)
        self.filtered_line, = self.ax.plot([], [], 'b-', linewidth=3, label='Filtered Odometry (EKF)', alpha=0.9)
        
        # Current position markers and arrows
        self.raw_marker, = self.ax.plot([], [], 'ro', markersize=10, label='Raw Current')
        self.filtered_marker, = self.ax.plot([], [], 'bo', markersize=10, label='Filtered Current')
        
        # Direction arrows (will be added dynamically)
        self.raw_arrow = None
        self.filtered_arrow = None
        
        # Move legend to upper left, smaller and more transparent
        self.ax.legend(loc='upper left', fontsize=8, framealpha=0.8, 
                      bbox_to_anchor=(0.02, 0.98))
        
        # Statistics text - move to right side and make smaller
        self.stats_text = self.ax.text(0.98, 0.5, '', transform=self.ax.transAxes, 
                                      verticalalignment='center', horizontalalignment='right',
                                      fontfamily='monospace', fontsize=7,
                                      bbox=dict(boxstyle='round,pad=0.2', facecolor='lightgray', alpha=0.8))
        
        # Add keyboard event handler
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == ' ':  # Space key to clear trajectories
            with self.data_lock:
                self.raw_odom_trajectory.clear()
                self.filtered_odom_trajectory.clear()
                self.raw_odom_full_trajectory.clear()
                self.filtered_odom_full_trajectory.clear()
                self.get_logger().info('Trajectories cleared - ready for new test run')

    def update_plot(self, frame):
        """Update the plot with new data"""
        try:
            with self.data_lock:
                # Update trajectory lines using full trajectory to prevent disappearing
                if len(self.raw_odom_full_trajectory) >= 1:
                    raw_x, raw_y = zip(*self.raw_odom_full_trajectory)
                    self.raw_line.set_data(raw_x, raw_y)
                    
                    # Update current position marker
                    self.raw_marker.set_data([self.raw_odom_current['x']], [self.raw_odom_current['y']])
                    
                    # Update direction arrow
                    if self.raw_arrow:
                        self.raw_arrow.remove()
                    self.raw_arrow = self.ax.annotate('', 
                        xy=(self.raw_odom_current['x'] + 0.4 * math.cos(self.raw_odom_current['yaw']),
                            self.raw_odom_current['y'] + 0.4 * math.sin(self.raw_odom_current['yaw'])),
                        xytext=(self.raw_odom_current['x'], self.raw_odom_current['y']),
                        arrowprops=dict(arrowstyle='->', color='red', lw=3))
                
                if len(self.filtered_odom_full_trajectory) >= 1:
                    filt_x, filt_y = zip(*self.filtered_odom_full_trajectory)
                    self.filtered_line.set_data(filt_x, filt_y)
                    
                    # Update current position marker
                    self.filtered_marker.set_data([self.filtered_odom_current['x']], [self.filtered_odom_current['y']])
                    
                    # Update direction arrow
                    if self.filtered_arrow:
                        self.filtered_arrow.remove()
                    self.filtered_arrow = self.ax.annotate('', 
                        xy=(self.filtered_odom_current['x'] + 0.4 * math.cos(self.filtered_odom_current['yaw']),
                            self.filtered_odom_current['y'] + 0.4 * math.sin(self.filtered_odom_current['yaw'])),
                        xytext=(self.filtered_odom_current['x'], self.filtered_odom_current['y']),
                        arrowprops=dict(arrowstyle='->', color='blue', lw=3))
                
                # Update statistics
                self.update_statistics()
                
                # Ensure plot limits accommodate the 5x10 rectangle and all data
                self.ax.set_xlim(-1, 6)
                self.ax.set_ylim(-1, 11)
                
                # Force canvas draw
                self.fig.canvas.draw_idle()
            
            return [self.raw_line, self.filtered_line, self.raw_marker, self.filtered_marker]
        except Exception as e:
            self.get_logger().error(f"Plot update error: {e}")
            return []

    def update_statistics(self):
        """Update the statistics display on the plot"""
        stats_text = "TRAJECTORY STATUS:\n"
        
        if self.raw_odom_trajectory and self.filtered_odom_trajectory:
            # Calculate difference
            dx = self.filtered_odom_current['x'] - self.raw_odom_current['x']
            dy = self.filtered_odom_current['y'] - self.raw_odom_current['y']
            distance_diff = math.sqrt(dx*dx + dy*dy)
            
            # Calculate trajectory lengths using full trajectories
            if len(self.raw_odom_full_trajectory) > 1:
                raw_path = self.raw_odom_full_trajectory
                raw_distance = sum(math.sqrt((raw_path[i][0] - raw_path[i-1][0])**2 + 
                                           (raw_path[i][1] - raw_path[i-1][1])**2) 
                                 for i in range(1, len(raw_path)))
                stats_text += f"Raw Distance: {raw_distance:.2f}m\n"
            
            if len(self.filtered_odom_full_trajectory) > 1:
                filt_path = self.filtered_odom_full_trajectory
                filt_distance = sum(math.sqrt((filt_path[i][0] - filt_path[i-1][0])**2 + 
                                            (filt_path[i][1] - filt_path[i-1][1])**2) 
                                  for i in range(1, len(filt_path)))
                stats_text += f"Filtered Distance: {filt_distance:.2f}m\n"
            
            stats_text += f"Position Diff: {distance_diff:.3f}m\n"
            stats_text += f"Raw Points: {len(self.raw_odom_full_trajectory)}\n"
            stats_text += f"Filtered Points: {len(self.filtered_odom_full_trajectory)}"
        else:
            stats_text += "Waiting for data...\n"
            stats_text += "Expected: 30m rectangle\n"
            stats_text += "(5+10+5+10 = 30m)"
        
        self.stats_text.set_text(stats_text)

    def terminal_display_loop(self):
        """Terminal data display loop running in separate thread"""
        while True:
            try:
                print("\n" + "="*100)
                print("WHEELCHAIR EKF TEST - REAL-TIME DATA")
                print("="*100)
                
                with self.data_lock:
                    # Raw Odometry
                    if self.raw_odom_data:
                        print("\n🔴 RAW ODOMETRY (/wc_control/odom):")
                        print(f"   Position: x={self.raw_odom_data.pose.pose.position.x:.3f}m, y={self.raw_odom_data.pose.pose.position.y:.3f}m")
                        yaw_deg = math.degrees(self.quaternion_to_yaw(self.raw_odom_data.pose.pose.orientation))
                        print(f"   Orientation: {yaw_deg:.1f}°")
                        print(f"   Linear velocity: {self.raw_odom_data.twist.twist.linear.x:.3f}m/s")
                        print(f"   Angular velocity: {math.degrees(self.raw_odom_data.twist.twist.angular.z):.1f}°/s")
                    else:
                        print("\n🔴 RAW ODOMETRY: No data")
                    
                    # Filtered Odometry
                    if self.filtered_odom_data:
                        print("\n🔵 FILTERED ODOMETRY (/odometry/filtered):")
                        print(f"   Position: x={self.filtered_odom_data.pose.pose.position.x:.3f}m, y={self.filtered_odom_data.pose.pose.position.y:.3f}m")
                        yaw_deg = math.degrees(self.quaternion_to_yaw(self.filtered_odom_data.pose.pose.orientation))
                        print(f"   Orientation: {yaw_deg:.1f}°")
                        print(f"   Linear velocity: {self.filtered_odom_data.twist.twist.linear.x:.3f}m/s")
                        print(f"   Angular velocity: {math.degrees(self.filtered_odom_data.twist.twist.angular.z):.1f}°/s")
                    else:
                        print("\n🔵 FILTERED ODOMETRY: No data")
                    
                    # IMU Data
                    if self.imu_data:
                        print("\n🟢 IMU DATA (/imu/odom_oriented):")
                        yaw_deg = math.degrees(self.quaternion_to_yaw(self.imu_data.orientation))
                        print(f"   Orientation: {yaw_deg:.1f}°")
                        print(f"   Angular velocity: x={math.degrees(self.imu_data.angular_velocity.x):.1f}°/s, "
                              f"y={math.degrees(self.imu_data.angular_velocity.y):.1f}°/s, "
                              f"z={math.degrees(self.imu_data.angular_velocity.z):.1f}°/s")
                        print(f"   Linear acceleration: x={self.imu_data.linear_acceleration.x:.3f}m/s², "
                              f"y={self.imu_data.linear_acceleration.y:.3f}m/s², "
                              f"z={self.imu_data.linear_acceleration.z:.3f}m/s²")
                    else:
                        print("\n🟢 IMU DATA: No data")
                    
                    # Comparison
                    if self.raw_odom_data and self.filtered_odom_data:
                        print("\nCOMPARISON (Filtered - Raw):")
                        dx = self.filtered_odom_data.pose.pose.position.x - self.raw_odom_data.pose.pose.position.x
                        dy = self.filtered_odom_data.pose.pose.position.y - self.raw_odom_data.pose.pose.position.y
                        distance_diff = math.sqrt(dx*dx + dy*dy)
                        print(f"   Position difference: dx={dx:.3f}m, dy={dy:.3f}m, total={distance_diff:.3f}m")
                        
                        yaw_raw = self.quaternion_to_yaw(self.raw_odom_data.pose.pose.orientation)
                        yaw_filtered = self.quaternion_to_yaw(self.filtered_odom_data.pose.pose.orientation)
                        yaw_diff = math.degrees(yaw_filtered - yaw_raw)
                        print(f"   Orientation difference: {yaw_diff:.1f}°")
                        
                        vel_diff = self.filtered_odom_data.twist.twist.linear.x - self.raw_odom_data.twist.twist.linear.x
                        angular_vel_diff = math.degrees(self.filtered_odom_data.twist.twist.angular.z - self.raw_odom_data.twist.twist.angular.z)
                        print(f"   Velocity difference: linear={vel_diff:.3f}m/s, angular={angular_vel_diff:.1f}°/s")
                
                print("\n💡 Use teleop_twist_keyboard in the simulation to move the wheelchair")
                print("   Use 'i' to move forward, 'j'/'l' to turn, 'k' to stop")
                
                time.sleep(2.0)  # Update every 2 seconds
                
            except Exception as e:
                print(f"Terminal display error: {e}")
                time.sleep(2.0)

    def cleanup_csv_files(self):
        """Close CSV files properly"""
        try:
            if self.csv_odom_file_handle:
                self.csv_odom_file_handle.close()
            if self.csv_imu_file_handle:
                self.csv_imu_file_handle.close()
            self.get_logger().info("CSV files closed successfully")
        except Exception as e:
            self.get_logger().error(f"Error closing CSV files: {e}")

    def start_visualization(self):
        """Start the real-time visualization"""
        # Start animation with faster update rate and ensure it keeps running
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)
        plt.ion()  # Turn on interactive mode
        plt.show()
        plt.pause(0.001)  # Small pause to ensure window opens

def main(args=None):
    rclpy.init(args=args)
    
    node = EKFRealtimeVisualizer()
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    try:
        # Start the visualization (this will block until window is closed)
        node.start_visualization()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup_csv_files()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()