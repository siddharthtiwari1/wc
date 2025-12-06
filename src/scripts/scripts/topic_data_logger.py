#!/usr/bin/env python3

"""
Log /imu, /camera/imu (raw), /wc_control/odom, and /odometry/filtered streams to a single CSV file.

Each row contains:
    - wall clock timestamp
    - latest IMU orientation/angular velocity/linear acceleration (transformed to base_link)
    - latest RAW camera IMU angular velocity (in camera_imu_optical_frame - for bias calibration)
    - latest raw odometry pose/twist (from wc_control)
    - latest EKF fused odometry pose/twist
"""

import csv
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def _stamp_to_float(msg_stamp) -> float:
    """Convert builtin_interfaces/Time to seconds as float."""
    return msg_stamp.sec + msg_stamp.nanosec * 1e-9


@dataclass
class ImuSnapshot:
    stamp: float
    orientation: tuple
    angular_velocity: tuple
    linear_acceleration: tuple


@dataclass
class RawImuSnapshot:
    """Raw camera IMU data (in camera_imu_optical_frame) for bias calibration."""
    stamp: float
    angular_velocity: tuple  # gyro X, Y, Z in camera frame
    linear_acceleration: tuple


@dataclass
class OdomSnapshot:
    stamp: float
    position: tuple
    orientation: tuple
    linear_velocity: tuple
    angular_velocity: tuple


class MultiTopicLogger(Node):
    def __init__(self) -> None:
        super().__init__('multi_topic_logger')

        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('raw_camera_imu_topic', '/camera/imu')  # Raw camera IMU for bias calibration
        self.declare_parameter('raw_odom_topic', '/wc_control/odom')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('log_frequency_hz', 10.0)
        self.declare_parameter('output_path', '')
        self.declare_parameter('file_prefix', 'imu_ekf_log')

        self._imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self._raw_camera_imu_topic = self.get_parameter('raw_camera_imu_topic').get_parameter_value().string_value
        self._raw_odom_topic = self.get_parameter('raw_odom_topic').get_parameter_value().string_value
        self._filtered_odom_topic = self.get_parameter('filtered_odom_topic').get_parameter_value().string_value
        log_frequency = self.get_parameter('log_frequency_hz').get_parameter_value().double_value
        self._log_period = 1.0 / max(log_frequency, 0.1)
        output_path = self.get_parameter('output_path').get_parameter_value().string_value
        file_prefix = self.get_parameter('file_prefix').get_parameter_value().string_value or 'imu_ekf_log'

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if output_path:
            self._csv_path = Path(output_path).expanduser()
        else:
            # Default to data_logs folder with timestamped filename
            log_dir = Path('/home/sidd/wc/src/data_logs')
            log_dir.mkdir(parents=True, exist_ok=True)
            self._csv_path = log_dir / f'{file_prefix}_{timestamp}.csv'
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu, self._imu_topic, self._imu_callback, qos)
        self.create_subscription(Imu, self._raw_camera_imu_topic, self._raw_camera_imu_callback, qos)
        self.create_subscription(Odometry, self._raw_odom_topic, self._raw_odom_callback, qos)
        self.create_subscription(Odometry, self._filtered_odom_topic, self._filtered_odom_callback, qos)

        self._imu_snapshot: Optional[ImuSnapshot] = None
        self._raw_camera_imu_snapshot: Optional[RawImuSnapshot] = None
        self._raw_odom_snapshot: Optional[OdomSnapshot] = None
        self._filtered_odom_snapshot: Optional[OdomSnapshot] = None

        self._csv_file = self._csv_path.open('w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        header = [
            'wall_time',
            'imu_stamp',
            'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
            'imu_ang_vel_x', 'imu_ang_vel_y', 'imu_ang_vel_z',
            'imu_lin_acc_x', 'imu_lin_acc_y', 'imu_lin_acc_z',
            # Raw camera IMU (in camera_imu_optical_frame - for bias calibration)
            'raw_cam_imu_stamp',
            'raw_cam_imu_ang_vel_x', 'raw_cam_imu_ang_vel_y', 'raw_cam_imu_ang_vel_z',
            'raw_cam_imu_lin_acc_x', 'raw_cam_imu_lin_acc_y', 'raw_cam_imu_lin_acc_z',
            'raw_stamp',
            'raw_pos_x', 'raw_pos_y', 'raw_pos_z',
            'raw_orientation_x', 'raw_orientation_y', 'raw_orientation_z', 'raw_orientation_w',
            'raw_lin_vel_x', 'raw_lin_vel_y', 'raw_lin_vel_z',
            'raw_ang_vel_x', 'raw_ang_vel_y', 'raw_ang_vel_z',
            'filtered_stamp',
            'filtered_pos_x', 'filtered_pos_y', 'filtered_pos_z',
            'filtered_orientation_x', 'filtered_orientation_y', 'filtered_orientation_z', 'filtered_orientation_w',
            'filtered_lin_vel_x', 'filtered_lin_vel_y', 'filtered_lin_vel_z',
            'filtered_ang_vel_x', 'filtered_ang_vel_y', 'filtered_ang_vel_z',
        ]
        self._csv_writer.writerow(header)
        self._csv_file.flush()
        self.get_logger().info(f'Logging IMU/Odometry data to {self._csv_path}')

        self._timer = self.create_timer(self._log_period, self._write_row)

    def _imu_callback(self, msg: Imu) -> None:
        self._imu_snapshot = ImuSnapshot(
            stamp=_stamp_to_float(msg.header.stamp),
            orientation=(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            angular_velocity=(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            linear_acceleration=(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
        )

    def _raw_camera_imu_callback(self, msg: Imu) -> None:
        """Capture raw camera IMU data for bias calibration."""
        self._raw_camera_imu_snapshot = RawImuSnapshot(
            stamp=_stamp_to_float(msg.header.stamp),
            angular_velocity=(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            linear_acceleration=(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
        )

    def _raw_odom_callback(self, msg: Odometry) -> None:
        self._raw_odom_snapshot = self._extract_odom(msg)

    def _filtered_odom_callback(self, msg: Odometry) -> None:
        self._filtered_odom_snapshot = self._extract_odom(msg)

    @staticmethod
    def _extract_odom(msg: Odometry) -> OdomSnapshot:
        pose = msg.pose.pose
        twist = msg.twist.twist
        return OdomSnapshot(
            stamp=_stamp_to_float(msg.header.stamp),
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation=(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            linear_velocity=(twist.linear.x, twist.linear.y, twist.linear.z),
            angular_velocity=(twist.angular.x, twist.angular.y, twist.angular.z),
        )

    def _write_row(self) -> None:
        if not (self._imu_snapshot and self._raw_odom_snapshot and self._filtered_odom_snapshot):
            return

        now = self.get_clock().now().nanoseconds / 1e9
        imu = self._imu_snapshot
        raw_cam = self._raw_camera_imu_snapshot
        raw = self._raw_odom_snapshot
        filt = self._filtered_odom_snapshot

        # Handle case where raw camera IMU is not yet available
        if raw_cam:
            raw_cam_data = [raw_cam.stamp, *raw_cam.angular_velocity, *raw_cam.linear_acceleration]
        else:
            raw_cam_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        row = [
            now,
            imu.stamp,
            *imu.orientation,
            *imu.angular_velocity,
            *imu.linear_acceleration,
            *raw_cam_data,
            raw.stamp,
            *raw.position,
            *raw.orientation,
            *raw.linear_velocity,
            *raw.angular_velocity,
            filt.stamp,
            *filt.position,
            *filt.orientation,
            *filt.linear_velocity,
            *filt.angular_velocity,
        ]
        self._csv_writer.writerow(row)
        self._csv_file.flush()

    def destroy_node(self) -> bool:
        if hasattr(self, '_csv_file'):
            self._csv_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiTopicLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Shutdown may already be in progress (e.g., global Ctrl+C)
            pass


if __name__ == '__main__':
    main()
