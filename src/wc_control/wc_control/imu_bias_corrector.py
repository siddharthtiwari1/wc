#!/usr/bin/env python3
"""
IMU Bias Corrector Node
========================
Applies gyro bias correction to raw IMU data BEFORE the Madgwick filter.

This fixes yaw drift at the source, so the Madgwick filter orientation
output will be drift-free.

Calibration values measured from static tests (latest 2025-12-06):
- gyro_z_bias: -0.0176 rad/s (updated using latest static log mean)

Subscribe: /camera/imu (raw IMU from RealSense)
Publish:   /camera/imu_corrected (bias-corrected IMU for Madgwick)

Author: Siddharth Tiwari
Date: 2025-12-05
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu


class ImuBiasCorrector(Node):
    """Apply gyro bias correction to raw IMU data."""

    def __init__(self):
        super().__init__('imu_bias_corrector')

        # Declare parameters with calibrated default values
        self.declare_parameter('input_topic', '/camera/imu')
        self.declare_parameter('output_topic', '/camera/imu_corrected')

        # Gyro bias values (static calibration 2025-12-06 17:38 from full_system_20251206_173829.csv)
        # DIRECT raw camera IMU measurements from 1044-second static test
        # These are applied to /camera/imu BEFORE Madgwick filter
        self.declare_parameter('gyro_x_bias', -0.004302)   # rad/s - measured raw camera gyro X mean
        self.declare_parameter('gyro_y_bias', 0.000787)    # rad/s - measured raw camera gyro Y mean
        self.declare_parameter('gyro_z_bias', 0.000948)    # rad/s - measured raw camera gyro Z mean

        # Get parameters
        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._gyro_x_bias = self.get_parameter('gyro_x_bias').get_parameter_value().double_value
        self._gyro_y_bias = self.get_parameter('gyro_y_bias').get_parameter_value().double_value
        self._gyro_z_bias = self.get_parameter('gyro_z_bias').get_parameter_value().double_value

        # Create publisher and subscriber
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._publisher = self.create_publisher(Imu, self._output_topic, qos)
        self._subscription = self.create_subscription(
            Imu, self._input_topic, self._imu_callback, qos
        )

        self.get_logger().info(
            f'IMU Bias Corrector started: {self._input_topic} -> {self._output_topic}\n'
            f'Gyro bias (rad/s): X={self._gyro_x_bias:.6f}, Y={self._gyro_y_bias:.6f}, Z={self._gyro_z_bias:.6f}'
        )

    def _imu_callback(self, msg: Imu):
        """Apply bias correction and republish."""
        corrected = Imu()
        corrected.header = msg.header

        # Copy orientation (unchanged)
        corrected.orientation = msg.orientation
        corrected.orientation_covariance = msg.orientation_covariance

        # Apply gyro bias correction
        corrected.angular_velocity.x = msg.angular_velocity.x - self._gyro_x_bias
        corrected.angular_velocity.y = msg.angular_velocity.y - self._gyro_y_bias
        corrected.angular_velocity.z = msg.angular_velocity.z - self._gyro_z_bias
        corrected.angular_velocity_covariance = msg.angular_velocity_covariance

        # Copy linear acceleration (unchanged)
        corrected.linear_acceleration = msg.linear_acceleration
        corrected.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self._publisher.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
