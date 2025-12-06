#!/usr/bin/env python3

"""
Republish IMU data from camera_imu_optical_frame to base_link-aligned 'imu' frame.

IMU Filter (Madgwick) outputs:
- Orientation: quaternion relative to world frame (odom/enu)
- Angular velocity: vector in camera_imu_optical_frame
- Linear acceleration: vector in camera_imu_optical_frame

This node transforms all data to base_link frame for robot localization/EKF using TF.
"""

from math import sqrt, atan2, asin
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import numpy as np

QuaternionTuple = Tuple[float, float, float, float]


def _normalize_quaternion(q: QuaternionTuple) -> QuaternionTuple:
    x, y, z, w = q
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def _quaternion_conjugate(q: QuaternionTuple) -> QuaternionTuple:
    x, y, z, w = q
    return -x, -y, -z, w


def _quaternion_multiply(a: QuaternionTuple, b: QuaternionTuple) -> QuaternionTuple:
    """Multiply two quaternions: result = a * b"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_to_rotation_matrix(q: QuaternionTuple) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix"""
    x, y, z, w = _normalize_quaternion(q)

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
    ])


def _rotate_vector(q: QuaternionTuple, vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Rotate a vector by quaternion q using rotation matrix"""
    R = _quat_to_rotation_matrix(q)
    v = np.array(vec)
    result = R @ v
    return tuple(result.tolist())


def _is_isotropic_covariance(cov: list, rtol: float = 1e-6) -> bool:
    """
    Check if a 3x3 covariance matrix is isotropic (diagonal with equal values).

    An isotropic covariance means the sensor has equal uncertainty in all directions.
    For such covariances, rotation doesn't change the matrix values.

    Args:
        cov: 9-element list (row-major 3x3 matrix)
        rtol: relative tolerance for comparing diagonal values

    Returns:
        True if covariance is isotropic (diagonal with equal values)
    """
    # Check if off-diagonal elements are effectively zero
    off_diag_threshold = 1e-9
    off_diagonal_elements = [cov[1], cov[2], cov[3], cov[5], cov[6], cov[7]]
    if not all(abs(x) < off_diag_threshold for x in off_diagonal_elements):
        return False

    # Check if diagonal elements are equal (within relative tolerance)
    diagonal_elements = [cov[0], cov[4], cov[8]]
    if any(x == 0.0 for x in diagonal_elements):
        # If any diagonal is zero, check if all are zero
        return all(x == 0.0 for x in diagonal_elements)

    # Check if all diagonal values are approximately equal
    avg_value = sum(diagonal_elements) / 3.0
    return all(abs(x - avg_value) / avg_value < rtol for x in diagonal_elements)


def _rotate_covariance(q: QuaternionTuple, cov: list) -> list:
    """
    Rotate a 3x3 covariance matrix using quaternion rotation.
    Formula: C' = R * C * R^T
    Input: 9-element list (row-major 3x3 matrix)
    Output: 9-element list (rotated covariance)
    """
    x, y, z, w = _normalize_quaternion(q)

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    R = [
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ]

    C = [[cov[i * 3 + j] for j in range(3)] for i in range(3)]
    RC = [[sum(R[i][k] * C[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    result = [[sum(RC[i][k] * R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    return [result[i][j] for i in range(3) for j in range(3)]


def _quaternion_to_rpy(q: QuaternionTuple) -> Tuple[float, float, float]:
    """
    Convert quaternion to Roll-Pitch-Yaw (ZYX convention).
    Returns (roll, pitch, yaw) in radians.
    """
    x, y, z, w = _normalize_quaternion(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = atan2(sinp, 0) if sinp >= 0 else -atan2(-sinp, 0)
    else:
        pitch = asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


class ImuWheelchairRepublisher(Node):
    """
    Transform IMU data from camera_imu_optical_frame to base_link frame using TF.

    Handles the fact that:
    - Orientation quaternion is relative to world (odom) frame
    - Angular velocity & linear acceleration are in camera_imu_optical_frame

    Subscribes to /imu/data and publishes to /imu (base_link aligned).
    """

    def __init__(self) -> None:
        super().__init__('imu_wheelchair_republisher')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu')
        # Publish in base_link frame so EKF doesn't need to re-transform
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('source_frame', 'camera_imu_optical_frame')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('zero_on_start', True)
        # Prefer TF to stay in sync with URDF; hardcoded quaternion is only a fallback
        self.declare_parameter('use_tf', True)

        # Hardcoded quaternion from URDF: camera_imu_optical_frame -> base_link
        # Format: [x, y, z, w]
        self.declare_parameter('orientation_quaternion', [0.5, -0.5, 0.5, 0.5])

        # Gyro bias correction in base_link frame
        self.declare_parameter('gyro_z_bias', 0.0)  # rad/s

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self._source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self._target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self._zero_on_start = self.get_parameter('zero_on_start').get_parameter_value().bool_value
        self._use_tf = self.get_parameter('use_tf').get_parameter_value().bool_value
        self._gyro_z_bias = self.get_parameter('gyro_z_bias').get_parameter_value().double_value

        # Get hardcoded quaternions
        ori_quat = self.get_parameter('orientation_quaternion').get_parameter_value().double_array_value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Initialize transform variables (will be set by TF lookup or hardcoded values)
        self._R_sensor_to_base: Optional[np.ndarray] = None
        self._q_sensor_to_base: Optional[QuaternionTuple] = None
        self._q_base_to_sensor: Optional[QuaternionTuple] = None

        # If not using TF, set up hardcoded quaternion immediately
        if not self._use_tf:
            # orientation_quaternion is sensor -> base_link (from URDF)
            self._q_sensor_to_base = tuple(ori_quat)
            self._q_base_to_sensor = _quaternion_conjugate(self._q_sensor_to_base)
            self._R_sensor_to_base = _quat_to_rotation_matrix(self._q_sensor_to_base)
            self.get_logger().info(
                f'Using hardcoded quaternions:\n'
                f'  Sensor->base quat: {self._q_sensor_to_base}\n'
                f'  Rotation matrix:\n{self._R_sensor_to_base}'
            )

        self._initial_orientation: Optional[QuaternionTuple] = None
        self._initial_orientation_inv: Optional[QuaternionTuple] = None
        self._message_count = 0
        self._last_log_time = None
        self._tf_warned = False

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._publisher = self.create_publisher(Imu, self._output_topic, qos)
        self._subscription = self.create_subscription(Imu, self._input_topic, self._handle_imu, qos)

        self.get_logger().info(
            f'Republishing IMU {self._input_topic} -> {self._output_topic} '
            f'using {"TF" if self._use_tf else "hardcoded quaternion"} to transform from {self._source_frame} to {self._target_frame}. '
            f'Gyro Z bias correction: {self._gyro_z_bias:.6f} rad/s'
        )

    def _lookup_transform(self) -> bool:
        """Lookup and cache the transform from sensor frame to base frame"""
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._source_frame,
                Time()
            )

            q = tf_stamped.transform.rotation
            # lookup_transform(target, source) returns the transform FROM source TO target
            # So this is sensor_to_base (camera_imu_optical_frame → base_link)
            self._q_sensor_to_base = (q.x, q.y, q.z, q.w)
            self._q_base_to_sensor = _quaternion_conjugate(self._q_sensor_to_base)
            self._R_sensor_to_base = _quat_to_rotation_matrix(self._q_sensor_to_base)

            if not self._tf_warned:
                self.get_logger().info(
                    f'TF lookup successful: {self._source_frame} -> {self._target_frame}\n'
                    f'Sensor->Base quat: [{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]\n'
                    f'Rotation matrix (sensor->base):\n{self._R_sensor_to_base}'
                )
                self._tf_warned = True

            return True

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if not self._tf_warned:
                self.get_logger().warn(
                    f'TF lookup failed: {self._source_frame} -> {self._target_frame}: {e}'
                )
                self._tf_warned = True
            return False

    def _handle_imu(self, msg: Imu) -> None:
        """Transform IMU data from sensor frame to base_link-aligned frame"""
        if self._use_tf and (self._R_sensor_to_base is None or self._q_sensor_to_base is None):
            if not self._lookup_transform():
                return

        if self._R_sensor_to_base is None or self._q_sensor_to_base is None:
            self.get_logger().warn('Transform not available, skipping message')
            return

        republished = Imu()
        republished.header = msg.header
        republished.header.frame_id = self._output_frame

        sensor_quat = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        base_orientation = _quaternion_multiply(sensor_quat, self._q_base_to_sensor)

        if self._zero_on_start and self._initial_orientation is None:
            self._initial_orientation = base_orientation
            self._initial_orientation_inv = _quaternion_conjugate(base_orientation)

        if self._zero_on_start and self._initial_orientation_inv is not None:
            aligned_orientation = _quaternion_multiply(self._initial_orientation_inv, base_orientation)
        else:
            aligned_orientation = base_orientation

        republished.orientation.x = aligned_orientation[0]
        republished.orientation.y = aligned_orientation[1]
        republished.orientation.z = aligned_orientation[2]
        republished.orientation.w = aligned_orientation[3]

        angular_sensor = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        if np.all(np.isfinite(angular_sensor)):
            angular_base = self._R_sensor_to_base @ angular_sensor
            republished.angular_velocity.x = float(angular_base[0])
            republished.angular_velocity.y = float(angular_base[1])
            # Apply gyro Z bias correction to reduce yaw drift
            republished.angular_velocity.z = float(angular_base[2]) - self._gyro_z_bias
        else:
            republished.angular_velocity = msg.angular_velocity

        linear_sensor = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        if np.all(np.isfinite(linear_sensor)):
            linear_base = self._R_sensor_to_base @ linear_sensor
            
            # Calibrate X acceleration for wheelchair:
            # 1. Invert direction (negative when moving forward)
            calibrated_x_accel = -linear_base[0]
            
            republished.linear_acceleration.x = float(calibrated_x_accel)
            republished.linear_acceleration.y = float(linear_base[1])
            republished.linear_acceleration.z = float(linear_base[2])
        else:
            republished.linear_acceleration = msg.linear_acceleration

        # Handle orientation covariance
        if all(np.isfinite(msg.orientation_covariance)):
            ori_cov = list(msg.orientation_covariance)
            if _is_isotropic_covariance(ori_cov):
                # If all zeros, provide a reasonable default (~3 deg std dev)
                if all(c == 0.0 for c in ori_cov):
                    orientation_variance = 0.0025  # (0.05 rad)^2
                    republished.orientation_covariance = [
                        orientation_variance, 0.0, 0.0,
                        0.0, orientation_variance, 0.0,
                        0.0, 0.0, orientation_variance
                    ]
                else:
                    republished.orientation_covariance = ori_cov
            else:
                republished.orientation_covariance = _rotate_covariance(
                    self._q_sensor_to_base, ori_cov
                )
        else:
            republished.orientation_covariance = msg.orientation_covariance

        # Handle angular velocity covariance
        if all(np.isfinite(msg.angular_velocity_covariance)):
            ang_vel_cov = list(msg.angular_velocity_covariance)
            if _is_isotropic_covariance(ang_vel_cov):
                republished.angular_velocity_covariance = ang_vel_cov
            else:
                republished.angular_velocity_covariance = _rotate_covariance(
                    self._q_sensor_to_base, ang_vel_cov
                )
        else:
            republished.angular_velocity_covariance = msg.angular_velocity_covariance

        # Handle linear acceleration covariance
        if all(np.isfinite(msg.linear_acceleration_covariance)):
            lin_acc_cov = list(msg.linear_acceleration_covariance)
            # If covariance is isotropic (diagonal with equal values), no rotation needed
            if _is_isotropic_covariance(lin_acc_cov):
                republished.linear_acceleration_covariance = lin_acc_cov
            else:
                # Rotate non-isotropic covariance using sensor->base quaternion
                republished.linear_acceleration_covariance = _rotate_covariance(
                    self._q_sensor_to_base, lin_acc_cov
                )
        else:
            republished.linear_acceleration_covariance = msg.linear_acceleration_covariance

        current_time = self.get_clock().now()
        if self._last_log_time is None:
            self._last_log_time = current_time

        time_diff = (current_time - self._last_log_time).nanoseconds / 1e9
        if time_diff >= 4.0:
            input_rpy = _quaternion_to_rpy(sensor_quat)
            input_roll_deg = input_rpy[0] * 180.0 / 3.14159265359
            input_pitch_deg = input_rpy[1] * 180.0 / 3.14159265359
            input_yaw_deg = input_rpy[2] * 180.0 / 3.14159265359

            output_rpy = _quaternion_to_rpy(aligned_orientation)
            output_roll_deg = output_rpy[0] * 180.0 / 3.14159265359
            output_pitch_deg = output_rpy[1] * 180.0 / 3.14159265359
            output_yaw_deg = output_rpy[2] * 180.0 / 3.14159265359

            self.get_logger().info(
                "\n" + "="*80 + "\n"
                "IMU DATA COMPARISON (Input → Output)\n" +
                "="*80 + "\n"
                "ORIENTATION (RPY in degrees):\n"
                f"  INPUT  Roll: {input_roll_deg:8.2f}°  Pitch: {input_pitch_deg:8.2f}°  Yaw: {input_yaw_deg:8.2f}°\n"
                f"  OUTPUT Roll: {output_roll_deg:8.2f}°  Pitch: {output_pitch_deg:8.2f}°  Yaw: {output_yaw_deg:8.2f}°\n"
                "\n"
                "LINEAR ACCELERATION (m/s²):\n"
                f"  INPUT  X: {linear_sensor[0]:7.3f}  Y: {linear_sensor[1]:7.3f}  Z: {linear_sensor[2]:7.3f}\n"
                f"  OUTPUT X: {linear_base[0]:7.3f}  Y: {linear_base[1]:7.3f}  Z: {linear_base[2]:7.3f}\n"
                "\n"
                "ANGULAR VELOCITY (rad/s):\n"
                f"  INPUT  X: {angular_sensor[0]:7.3f}  Y: {angular_sensor[1]:7.3f}  Z: {angular_sensor[2]:7.3f}\n"
                f"  OUTPUT X: {angular_base[0]:7.3f}  Y: {angular_base[1]:7.3f}  Z: {angular_base[2]:7.3f}\n" +
                "="*80
            )
            self._last_log_time = current_time

        self._publisher.publish(republished)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuWheelchairRepublisher()
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
