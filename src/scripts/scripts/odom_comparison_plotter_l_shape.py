#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for better stability
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

class OdomComparisonLShapePlotter(Node):
    def __init__(self):
        super().__init__('odom_comparison_plotter_l_shape')
        
        # Data storage for trajectories
        self.max_points = 5000
        self.raw_odom_trajectory = []
        self.filtered_odom_trajectory = []
        
        # Current positions and orientations
        self.raw_odom_current = {'x': 0, 'y': 0, 'yaw': 0, 'timestamp': 0}
        self.filtered_odom_current = {'x': 0, 'y': 0, 'yaw': 0, 'timestamp': 0}
        
        # Data for CSV logging
        self.raw_odom_data = None
        self.filtered_odom_data = None
        
        # CSV logging setup
        self.setup_csv_logging()

        # L-shape configuration
        self.l_leg_length = 5.5
        self.l_corridor_width = self.declare_parameter('l_corridor_width', 1.0).value
        if self.l_corridor_width <= 0.0:
            self.get_logger().warning('l_corridor_width must be positive; defaulting to 1.0 meters')
            self.l_corridor_width = 1.0
        self.get_logger().info(
            f'Reference L-shape configured: legs={self.l_leg_length:.2f}m, corridor width={self.l_corridor_width:.2f}m'
        )
        
        # Thread locks
        self.data_lock = threading.Lock()
        
        # ROS2 Subscribers
        self.raw_odom_sub = self.create_subscription(
            Odometry, '/wc_control/odom', self.raw_odom_callback, 10)
        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)
        
        # Setup matplotlib
        self.setup_plot()
        
        # Start terminal data display thread
        self.terminal_thread = threading.Thread(target=self.terminal_display_loop, daemon=True)
        self.terminal_thread.start()
        
        self.get_logger().info('Odometry Comparison Plotter (L-Shape) started')
        self.get_logger().info('Topics: /wc_control/odom (Red) vs /odometry/filtered (Blue)')
        self.get_logger().info('Press SPACE in plot window to clear trajectories')
        self.get_logger().info(f'CSV file: {self.csv_file}')

    def setup_csv_logging(self):
        """Setup CSV file for data logging"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f"odom_comparison_l_shape_{timestamp}.csv"
        self.csv_writer = None
        self.csv_file_handle = None
        
        # Initialize CSV file
        self.init_csv()
    
    def init_csv(self):
        """Initialize the CSV file"""
        try:
            self.csv_file_handle = open(self.csv_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file_handle)
            
            # Write header
            header = [
                'timestamp', 
                'raw_odom_x', 'raw_odom_y', 'raw_odom_z',
                'raw_odom_roll', 'raw_odom_pitch', 'raw_odom_yaw',
                'raw_odom_vel_x', 'raw_odom_vel_y', 'raw_odom_vel_z',
                'raw_odom_vel_roll', 'raw_odom_vel_pitch', 'raw_odom_vel_yaw',
                'filtered_odom_x', 'filtered_odom_y', 'filtered_odom_z',
                'filtered_odom_roll', 'filtered_odom_pitch', 'filtered_odom_yaw',
                'filtered_odom_vel_x', 'filtered_odom_vel_y', 'filtered_odom_vel_z',
                'filtered_odom_vel_roll', 'filtered_odom_vel_pitch', 'filtered_odom_vel_yaw'
            ]
            self.csv_writer.writerow(header)
            self.csv_file_handle.flush()
            
            # Verify file was created
            if os.path.exists(self.csv_file):
                self.get_logger().info(f"✅ CSV file created successfully: {self.csv_file}")
            else:
                self.get_logger().error(f"❌ CSV file creation failed: {self.csv_file}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to create CSV file: {e}")
    
    def log_data(self):
        """Log odometry comparison data to CSV"""
        if self.csv_writer and self.raw_odom_data and self.filtered_odom_data:
            try:
                current_time = time.time()
                
                # Raw odom data
                raw_pos = self.raw_odom_data.pose.pose.position
                raw_orient = self.quaternion_to_euler(self.raw_odom_data.pose.pose.orientation)
                raw_vel_linear = self.raw_odom_data.twist.twist.linear
                raw_vel_angular = self.raw_odom_data.twist.twist.angular
                
                # Filtered odom data
                filt_pos = self.filtered_odom_data.pose.pose.position
                filt_orient = self.quaternion_to_euler(self.filtered_odom_data.pose.pose.orientation)
                filt_vel_linear = self.filtered_odom_data.twist.twist.linear
                filt_vel_angular = self.filtered_odom_data.twist.twist.angular
                
                # Write row
                row = [
                    current_time,
                    raw_pos.x, raw_pos.y, raw_pos.z,
                    raw_orient[0], raw_orient[1], raw_orient[2],
                    raw_vel_linear.x, raw_vel_linear.y, raw_vel_linear.z,
                    raw_vel_angular.x, raw_vel_angular.y, raw_vel_angular.z,
                    filt_pos.x, filt_pos.y, filt_pos.z,
                    filt_orient[0], filt_orient[1], filt_orient[2],
                    filt_vel_linear.x, filt_vel_linear.y, filt_vel_linear.z,
                    filt_vel_angular.x, filt_vel_angular.y, filt_vel_angular.z
                ]
                self.csv_writer.writerow(row)
                self.csv_file_handle.flush()
            except Exception as e:
                self.get_logger().error(f"Failed to log data: {e}")

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

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
            
            # Keep only recent points
            if len(self.raw_odom_trajectory) > self.max_points:
                self.raw_odom_trajectory = self.raw_odom_trajectory[-self.max_points:]
            
            # Log to CSV
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
            
            # Keep only recent points
            if len(self.filtered_odom_trajectory) > self.max_points:
                self.filtered_odom_trajectory = self.filtered_odom_trajectory[-self.max_points:]
            
            # Log to CSV
            self.log_data()

    def setup_plot(self):
        """Setup the matplotlib plot"""
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_xlabel('X Position (meters)', fontsize=12)
        self.ax.set_ylabel('Y Position (meters)', fontsize=12)
        self.ax.set_title('Odometry Comparison (L-Shape Reference): /wc_control/odom vs /odometry/filtered', fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Reference L-shape (5.5m x 5.5m legs) drawn as a 1D path
        l_shape_points = [
            (0.0, 0.0),
            (self.l_leg_length, 0.0),
            (self.l_leg_length, self.l_corridor_width),
            (self.l_corridor_width, self.l_corridor_width),
            (self.l_corridor_width, self.l_leg_length),
        ]
        l_shape_x, l_shape_y = zip(*l_shape_points)
        (self.l_shape_line,) = self.ax.plot(
            l_shape_x,
            l_shape_y,
            linestyle='--',
            color='black',
            linewidth=2.0,
            label=f'Reference L-Shape (legs {self.l_leg_length}m, width {self.l_corridor_width}m)',
        )
        self.ax.set_xlim(-1.0, self.l_leg_length + 1.0)
        self.ax.set_ylim(-1.0, self.l_leg_length + 1.0)
        
        # Initialize empty plots
        self.raw_line, = self.ax.plot([], [], 'r-', linewidth=2, label='/wc_control/odom (Raw)', alpha=0.8)
        self.filtered_line, = self.ax.plot([], [], 'b-', linewidth=2, label='/odometry/filtered (EKF)', alpha=0.8)
        
        # Current position markers
        self.raw_marker, = self.ax.plot([], [], 'ro', markersize=8, label='Raw Current')
        self.filtered_marker, = self.ax.plot([], [], 'bo', markersize=8, label='Filtered Current')
        
        # Direction arrows
        self.raw_arrow = None
        self.filtered_arrow = None
        
        # Legend
        self.ax.legend(loc='upper right', fontsize=10, framealpha=0.9)
        
        # Statistics text
        self.stats_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, 
                                      verticalalignment='top', horizontalalignment='left',
                                      fontfamily='monospace', fontsize=9,
                                      bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgray', alpha=0.8))
        
        # Add keyboard event handler
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Add close event handler to prevent accidental closing
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == ' ':  # Space key to clear trajectories
            with self.data_lock:
                self.raw_odom_trajectory.clear()
                self.filtered_odom_trajectory.clear()
                self.get_logger().info('Trajectories cleared!')
    
    def on_close(self, event):
        """Handle window close event"""
        self.get_logger().info('Plot window closed by user')
        # Don't prevent closing, just log it

    def update_plot(self, frame):
        """Update the plot with new data"""
        try:
            with self.data_lock:
                # Update trajectory lines
                if len(self.raw_odom_trajectory) >= 1:
                    raw_x, raw_y = zip(*self.raw_odom_trajectory)
                    self.raw_line.set_data(raw_x, raw_y)
                    
                    # Update current position marker
                    self.raw_marker.set_data([self.raw_odom_current['x']], [self.raw_odom_current['y']])
                    
                    # Update direction arrow
                    if self.raw_arrow:
                        self.raw_arrow.remove()
                    self.raw_arrow = self.ax.annotate('', 
                        xy=(self.raw_odom_current['x'] + 0.5 * math.cos(self.raw_odom_current['yaw']),
                            self.raw_odom_current['y'] + 0.5 * math.sin(self.raw_odom_current['yaw'])),
                        xytext=(self.raw_odom_current['x'], self.raw_odom_current['y']),
                        arrowprops=dict(arrowstyle='->', color='red', lw=2))
                
                if len(self.filtered_odom_trajectory) >= 1:
                    filt_x, filt_y = zip(*self.filtered_odom_trajectory)
                    self.filtered_line.set_data(filt_x, filt_y)
                    
                    # Update current position marker
                    self.filtered_marker.set_data([self.filtered_odom_current['x']], [self.filtered_odom_current['y']])
                    
                    # Update direction arrow
                    if self.filtered_arrow:
                        self.filtered_arrow.remove()
                    self.filtered_arrow = self.ax.annotate('', 
                        xy=(self.filtered_odom_current['x'] + 0.5 * math.cos(self.filtered_odom_current['yaw']),
                            self.filtered_odom_current['y'] + 0.5 * math.sin(self.filtered_odom_current['yaw'])),
                        xytext=(self.filtered_odom_current['x'], self.filtered_odom_current['y']),
                        arrowprops=dict(arrowstyle='->', color='blue', lw=2))
                
                # Update statistics
                self.update_statistics()
                
                # Keep axes focused around the reference L-shape with margin
                margin = 1.0
                min_x = min([0.0, self.raw_odom_current['x'], self.filtered_odom_current['x']]) - margin
                max_x = max([self.l_leg_length, self.raw_odom_current['x'], self.filtered_odom_current['x']]) + margin
                min_y = min([0.0, self.raw_odom_current['y'], self.filtered_odom_current['y']]) - margin
                max_y = max([self.l_leg_length, self.raw_odom_current['y'], self.filtered_odom_current['y']]) + margin
                self.ax.set_xlim(min_x, max_x)
                self.ax.set_ylim(min_y, max_y)
                
                # Force canvas draw
                self.fig.canvas.draw_idle()
            
            return [self.raw_line, self.filtered_line, self.raw_marker, self.filtered_marker]
        except Exception as e:
            self.get_logger().error(f"Plot update error: {e}")
            return []

    def update_statistics(self):
        """Update the statistics display on the plot"""
        stats_text = "ODOMETRY COMPARISON:\n"
        
        if self.raw_odom_trajectory and self.filtered_odom_trajectory:
            # Calculate difference
            dx = self.filtered_odom_current['x'] - self.raw_odom_current['x']
            dy = self.filtered_odom_current['y'] - self.raw_odom_current['y']
            distance_diff = math.sqrt(dx*dx + dy*dy)
            
            # Calculate trajectory lengths
            if len(self.raw_odom_trajectory) > 1:
                raw_path = self.raw_odom_trajectory
                raw_distance = sum(math.sqrt((raw_path[i][0] - raw_path[i-1][0])**2 + 
                                           (raw_path[i][1] - raw_path[i-1][1])**2) 
                                 for i in range(1, len(raw_path)))
                stats_text += f"Raw Distance: {raw_distance:.2f}m\n"
            
            if len(self.filtered_odom_trajectory) > 1:
                filt_path = self.filtered_odom_trajectory
                filt_distance = sum(math.sqrt((filt_path[i][0] - filt_path[i-1][0])**2 + 
                                            (filt_path[i][1] - filt_path[i-1][1])**2) 
                                  for i in range(1, len(filt_path)))
                stats_text += f"Filtered Distance: {filt_distance:.2f}m\n"
            
            stats_text += f"Position Diff: {distance_diff:.3f}m\n"
            stats_text += f"Raw Points: {len(self.raw_odom_trajectory)}\n"
            stats_text += f"Filtered Points: {len(self.filtered_odom_trajectory)}\n"
            
            # Current positions
            stats_text += f"\nCurrent Positions:\n"
            stats_text += f"Raw: ({self.raw_odom_current['x']:.2f}, {self.raw_odom_current['y']:.2f})\n"
            stats_text += f"Filtered: ({self.filtered_odom_current['x']:.2f}, {self.filtered_odom_current['y']:.2f})"
        else:
            stats_text += "Waiting for data...\n"
            stats_text += "Topics:\n"
            stats_text += "• /wc_control/odom\n"
            stats_text += "• /odometry/filtered"
        
        self.stats_text.set_text(stats_text)

    def terminal_display_loop(self):
        """Terminal data display loop running in separate thread"""
        while True:
            try:
                print("\n" + "="*80)
                print("ODOMETRY COMPARISON - REAL-TIME DATA")
                print("="*80)
                
                with self.data_lock:
                    # Raw Odometry
                    if self.raw_odom_data:
                        print("\n🔴 RAW ODOMETRY (/wc_control/odom):")
                        pos = self.raw_odom_data.pose.pose.position
                        print(f"   Position: x={pos.x:.3f}m, y={pos.y:.3f}m, z={pos.z:.3f}m")
                        roll, pitch, yaw = self.quaternion_to_euler(self.raw_odom_data.pose.pose.orientation)
                        print(f"   Orientation: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")
                        vel = self.raw_odom_data.twist.twist.linear
                        print(f"   Linear velocity: x={vel.x:.3f}m/s, y={vel.y:.3f}m/s, z={vel.z:.3f}m/s")
                        ang_vel = self.raw_odom_data.twist.twist.angular
                        print(f"   Angular velocity: x={math.degrees(ang_vel.x):.1f}°/s, y={math.degrees(ang_vel.y):.1f}°/s, z={math.degrees(ang_vel.z):.1f}°/s")
                    else:
                        print("\n🔴 RAW ODOMETRY: No data")
                    
                    # Filtered Odometry
                    if self.filtered_odom_data:
                        print("\n🔵 FILTERED ODOMETRY (/odometry/filtered):")
                        pos = self.filtered_odom_data.pose.pose.position
                        print(f"   Position: x={pos.x:.3f}m, y={pos.y:.3f}m, z={pos.z:.3f}m")
                        roll, pitch, yaw = self.quaternion_to_euler(self.filtered_odom_data.pose.pose.orientation)
                        print(f"   Orientation: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")
                        vel = self.filtered_odom_data.twist.twist.linear
                        print(f"   Linear velocity: x={vel.x:.3f}m/s, y={vel.y:.3f}m/s, z={vel.z:.3f}m/s")
                        ang_vel = self.filtered_odom_data.twist.twist.angular
                        print(f"   Angular velocity: x={math.degrees(ang_vel.x):.1f}°/s, y={math.degrees(ang_vel.y):.1f}°/s, z={math.degrees(ang_vel.z):.1f}°/s")
                    else:
                        print("\n🔵 FILTERED ODOMETRY: No data")
                    
                    # Comparison
                    if self.raw_odom_data and self.filtered_odom_data:
                        print("\nCOMPARISON (Filtered - Raw):")
                        raw_pos = self.raw_odom_data.pose.pose.position
                        filt_pos = self.filtered_odom_data.pose.pose.position
                        dx = filt_pos.x - raw_pos.x
                        dy = filt_pos.y - raw_pos.y
                        dz = filt_pos.z - raw_pos.z
                        distance_diff = math.sqrt(dx*dx + dy*dy + dz*dz)
                        print(f"   Position difference: dx={dx:.3f}m, dy={dy:.3f}m, dz={dz:.3f}m, total={distance_diff:.3f}m")
                
                print(f"\n💾 CSV file: {self.csv_file}")
                print("💡 Press SPACE in plot window to clear trajectories")
                
                time.sleep(2.0)  # Update every 2 seconds
                
            except Exception as e:
                print(f"Terminal display error: {e}")
                time.sleep(2.0)

    def cleanup_csv_files(self):
        """Close CSV files properly"""
        try:
            if self.csv_file_handle:
                self.csv_file_handle.close()
            self.get_logger().info("CSV file closed successfully")
        except Exception as e:
            self.get_logger().error(f"Error closing CSV file: {e}")

    def start_visualization(self):
        """Start the real-time visualization"""
        # Start animation with faster update rate
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)
        plt.ion()  # Turn on interactive mode
        plt.show(block=False)  # Non-blocking show
        
        # Keep the plot alive
        try:
            while plt.get_fignums():  # While figure windows exist
                plt.pause(0.1)  # Small pause to allow updates
                if not plt.fignum_exists(self.fig.number):
                    break
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = OdomComparisonLShapePlotter()
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    try:
        print("🚀 Starting visualization... Keep the plot window focused!")
        print("Plot will show real-time odometry comparison against 5.5m L-shape reference")
        print("⌨️  Press Ctrl+C in terminal to stop")
        
        # Start the visualization (this will block until window is closed)
        node.start_visualization()
        
        # If visualization ends, keep ROS spinning
        print("Plot window closed, but ROS node still running...")
        print("💾 CSV logging continues in background")
        print("⌨️  Press Ctrl+C to stop completely")
        
        # Keep the node alive for CSV logging even if plot closes
        while rclpy.ok():
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
    finally:
        node.cleanup_csv_files()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
