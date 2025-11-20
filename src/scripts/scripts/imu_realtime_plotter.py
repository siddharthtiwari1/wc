#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import threading
import queue


class IMURealtimePlotter(Node):
    """
    Real-time IMU data plotter showing multiple colored lines for:
    - Linear acceleration (X, Y, Z)
    - Angular velocity (X, Y, Z) 
    - Orientation (converted to Euler angles)
    """
    
    def __init__(self):
        super().__init__('imu_realtime_plotter')
        
        # Data storage (keep last 1000 points ~ 10 seconds at 100Hz)
        self.max_points = 1000
        self.time_data = deque(maxlen=self.max_points)
        
        # Linear acceleration data
        self.accel_x = deque(maxlen=self.max_points)
        self.accel_y = deque(maxlen=self.max_points) 
        self.accel_z = deque(maxlen=self.max_points)
        
        # Angular velocity data
        self.gyro_x = deque(maxlen=self.max_points)
        self.gyro_y = deque(maxlen=self.max_points)
        self.gyro_z = deque(maxlen=self.max_points)
        
        # Orientation data (Euler angles)
        self.roll = deque(maxlen=self.max_points)
        self.pitch = deque(maxlen=self.max_points)
        self.yaw = deque(maxlen=self.max_points)
        
        # Thread synchronization
        self.data_lock = threading.Lock()
        self.start_time = None
        
        # ROS2 subscriber
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/odom_oriented',  # Your main IMU topic
            self.imu_callback, 
            10
        )
        
        # Setup matplotlib
        self.setup_plot()
        
        self.get_logger().info("IMU Real-time Plotter started")
        self.get_logger().info("Subscribing to: /imu/odom_oriented")
        self.get_logger().info("Plotting 6-axis IMU data with color coding")
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def imu_callback(self, msg):
        """Process incoming IMU data"""
        with self.data_lock:
            # Initialize start time
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            # Calculate relative time
            current_time = self.get_clock().now()
            time_sec = (current_time - self.start_time).nanoseconds / 1e9
            self.time_data.append(time_sec)
            
            # Store linear acceleration
            self.accel_x.append(msg.linear_acceleration.x)
            self.accel_y.append(msg.linear_acceleration.y)
            self.accel_z.append(msg.linear_acceleration.z)
            
            # Store angular velocity
            self.gyro_x.append(msg.angular_velocity.x)
            self.gyro_y.append(msg.angular_velocity.y)
            self.gyro_z.append(msg.angular_velocity.z)
            
            # Convert quaternion to Euler and store
            roll, pitch, yaw = self.quaternion_to_euler(
                msg.orientation.x, msg.orientation.y, 
                msg.orientation.z, msg.orientation.w
            )
            self.roll.append(roll)
            self.pitch.append(pitch)
            self.yaw.append(yaw)
    
    def setup_plot(self):
        """Setup the matplotlib figure with subplots"""
        # Create figure with 3 subplots (2x2 layout, use 3)
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('Real-time IMU Data Visualization', fontsize=16, fontweight='bold')
        
        # Subplot 1: Linear Acceleration
        self.ax1.set_title('Linear Acceleration', fontsize=14, fontweight='bold')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Acceleration (m/s²)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend(['Accel X', 'Accel Y', 'Accel Z'], loc='upper right')
        
        # Subplot 2: Angular Velocity  
        self.ax2.set_title('Angular Velocity', fontsize=14, fontweight='bold')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Angular Velocity (rad/s)')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend(['Gyro X', 'Gyro Y', 'Gyro Z'], loc='upper right')
        
        # Subplot 3: Orientation (Euler Angles)
        self.ax3.set_title('Orientation (Euler Angles)', fontsize=14, fontweight='bold')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Angle (rad)')
        self.ax3.grid(True, alpha=0.3)
        self.ax3.legend(['Roll', 'Pitch', 'Yaw'], loc='upper right')
        
        # Subplot 4: Combined Magnitude View
        self.ax4.set_title('Combined Magnitude', fontsize=14, fontweight='bold')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Magnitude')
        self.ax4.grid(True, alpha=0.3)
        self.ax4.legend(['Total Accel', 'Total Gyro'], loc='upper right')
        
        # Initialize empty line objects
        self.init_lines()
        
        # Setup animation
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=100, blit=False
        )
    
    def init_lines(self):
        """Initialize line objects for each plot"""
        # Acceleration lines (Red, Green, Blue)
        self.accel_x_line, = self.ax1.plot([], [], 'r-', linewidth=2, label='Accel X')
        self.accel_y_line, = self.ax1.plot([], [], 'g-', linewidth=2, label='Accel Y')  
        self.accel_z_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='Accel Z')
        
        # Gyro lines (Dark Red, Dark Green, Dark Blue)
        self.gyro_x_line, = self.ax2.plot([], [], 'darkred', linewidth=2, label='Gyro X')
        self.gyro_y_line, = self.ax2.plot([], [], 'darkgreen', linewidth=2, label='Gyro Y')
        self.gyro_z_line, = self.ax2.plot([], [], 'darkblue', linewidth=2, label='Gyro Z')
        
        # Orientation lines (Orange, Purple, Cyan)
        self.roll_line, = self.ax3.plot([], [], 'orange', linewidth=2, label='Roll')
        self.pitch_line, = self.ax3.plot([], [], 'purple', linewidth=2, label='Pitch')
        self.yaw_line, = self.ax3.plot([], [], 'cyan', linewidth=2, label='Yaw')
        
        # Magnitude lines (Magenta, Brown)
        self.total_accel_line, = self.ax4.plot([], [], 'magenta', linewidth=3, label='Total Accel')
        self.total_gyro_line, = self.ax4.plot([], [], 'brown', linewidth=3, label='Total Gyro')
    
    def animate(self, frame):
        """Animation function called periodically"""
        with self.data_lock:
            if len(self.time_data) < 2:
                return
            
            # Convert deques to lists for plotting
            time_list = list(self.time_data)
            
            # Update acceleration plots
            self.accel_x_line.set_data(time_list, list(self.accel_x))
            self.accel_y_line.set_data(time_list, list(self.accel_y))
            self.accel_z_line.set_data(time_list, list(self.accel_z))
            
            # Update gyro plots
            self.gyro_x_line.set_data(time_list, list(self.gyro_x))
            self.gyro_y_line.set_data(time_list, list(self.gyro_y))
            self.gyro_z_line.set_data(time_list, list(self.gyro_z))
            
            # Update orientation plots
            self.roll_line.set_data(time_list, list(self.roll))
            self.pitch_line.set_data(time_list, list(self.pitch))
            self.yaw_line.set_data(time_list, list(self.yaw))
            
            # Calculate and update magnitudes
            if len(self.accel_x) > 0:
                total_accel = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in 
                              zip(self.accel_x, self.accel_y, self.accel_z)]
                total_gyro = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in 
                             zip(self.gyro_x, self.gyro_y, self.gyro_z)]
                
                self.total_accel_line.set_data(time_list, total_accel)
                self.total_gyro_line.set_data(time_list, total_gyro)
            
            # Auto-scale axes
            self.auto_scale_axes()
        
        return (self.accel_x_line, self.accel_y_line, self.accel_z_line,
                self.gyro_x_line, self.gyro_y_line, self.gyro_z_line,
                self.roll_line, self.pitch_line, self.yaw_line,
                self.total_accel_line, self.total_gyro_line)
    
    def auto_scale_axes(self):
        """Auto-scale all axes based on current data"""
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.relim()
            ax.autoscale_view()
    
    def show_plot(self):
        """Show the plot (blocking call)"""
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    
    plotter = IMURealtimePlotter()
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(plotter,))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Show the plot (this blocks)
        plotter.show_plot()
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()