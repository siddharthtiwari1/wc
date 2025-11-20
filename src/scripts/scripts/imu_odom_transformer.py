#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import tf2_ros
import tf_transformations
import numpy as np


class IMUOdomTransformer(Node):
    """
    Transforms IMU data from camera optical frame to odom frame orientation.
    
    This is crucial for EKF sensor fusion as all sensors should be in the same frame.
    
    Input: /imu_sensor_broadcaster/imu (camera_imu_optical_frame)
    Output: /imu/odom_oriented (odom frame orientation)
    """
    
    def __init__(self):
        super().__init__('imu_odom_transformer')
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers and Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/out', 
            self.imu_callback, 
            10
        )
        
        self.imu_pub = self.create_publisher(
            Imu, 
            '/imu/odom_oriented', 
            10
        )
        
        # Cache for transform
        self.transform_cached = False
        self.camera_to_odom_transform = None
        
        self.get_logger().info("IMU Odom Transformer started")
        self.get_logger().info("Input: /imu/out (any frame)")
        self.get_logger().info("Output: /imu/odom_oriented (odom frame orientation)")
    
    def get_transform_camera_to_odom(self):
        """Get transform from camera_imu_optical_frame to odom frame"""
        try:
            # Get transform from camera IMU optical frame to odom frame
            transform = self.tf_buffer.lookup_transform(
                'odom',  # target frame
                'camera_imu_optical_frame',  # source frame  
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return transform
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            return None
    
    def transform_imu_data(self, imu_msg):
        """Transform IMU data from camera optical frame to base_link frame for EKF use"""
        
        # Use a static transform since TF tree is incomplete
        # Camera IMU optical frame to base_link transformation
        # Camera optical frame: x=right, y=down, z=forward  
        # Base_link frame: x=forward, y=left, z=up
        # Need rotation to align: x->z, y->-x, z->y
        import math
        
        # Static rotation from camera_imu_optical_frame to base_link  
        # RealSense D455 IMU optical frame to robot base_link alignment
        # Based on the orientation difference, we need to correct the transform
        # Try identity first to see if issue is elsewhere
        roll, pitch, yaw = 0.0, 0.0, 0.0  # Identity transform to debug
        q_transform = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        
        # Create output message with synchronized timestamp
        transformed_imu = Imu()
        transformed_imu.header = imu_msg.header
        transformed_imu.header.frame_id = 'base_link'  # EKF expects base_link frame
        
        # Preserve original timestamp for simulation time synchronization
        # This ensures IMU and odometry have matching simulation time
        transformed_imu.header.stamp = imu_msg.header.stamp
        
        # Transform orientation
        q_original = [
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ]
        
        # Apply transform: q_result = q_transform * q_original
        q_result = tf_transformations.quaternion_multiply(q_transform, q_original)
        
        transformed_imu.orientation.x = q_result[0]
        transformed_imu.orientation.y = q_result[1]
        transformed_imu.orientation.z = q_result[2]
        transformed_imu.orientation.w = q_result[3]
        
        # Transform angular velocity (rotate vector)
        ang_vel_array = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        
        # Convert quaternion to rotation matrix and apply
        rotation_matrix = tf_transformations.quaternion_matrix(q_transform)[:3, :3]
        ang_vel_transformed = rotation_matrix.dot(ang_vel_array)
        
        transformed_imu.angular_velocity.x = ang_vel_transformed[0]
        transformed_imu.angular_velocity.y = ang_vel_transformed[1]
        transformed_imu.angular_velocity.z = ang_vel_transformed[2]
        
        # Transform linear acceleration (rotate vector)
        lin_acc_array = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        
        lin_acc_transformed = rotation_matrix.dot(lin_acc_array)
        
        transformed_imu.linear_acceleration.x = lin_acc_transformed[0]
        transformed_imu.linear_acceleration.y = lin_acc_transformed[1]
        transformed_imu.linear_acceleration.z = lin_acc_transformed[2]
        
        # Copy covariances
        transformed_imu.orientation_covariance = imu_msg.orientation_covariance
        transformed_imu.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        transformed_imu.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        
        return transformed_imu
    
    def imu_callback(self, msg):
        """Callback for incoming IMU data"""
        try:
            transformed_imu = self.transform_imu_data(msg)
            if transformed_imu is not None:
                self.imu_pub.publish(transformed_imu)
        except Exception as e:
            self.get_logger().error(f"Error transforming IMU data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUOdomTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()