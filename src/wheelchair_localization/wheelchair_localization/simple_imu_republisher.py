#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class SimpleImuRepublisher(Node):
    def __init__(self):
        super().__init__('simple_imu_republisher')
        
        # Simple republisher - just change frame_id like bumperbot
        self.imu_pub = self.create_publisher(Imu, "imu_ekf", 10)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        
        self.get_logger().info("Simple IMU republisher started - bumperbot style")
    
    def imu_callback(self, msg):
        # Simple approach: just change frame_id (like bumperbot)
        msg.header.frame_id = "base_footprint_ekf"  # FIXED: Match EKF base_link_frame
        self.imu_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleImuRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()