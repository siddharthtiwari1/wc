#!/usr/bin/env python3

"""
Republish /odom transformed from odom frame to base_link frame.

Input: /odom - odometry with poses in odom frame (world-fixed)
Output: /odom/camera - odometry transformed to base_link frame

The input odometry contains:
- pose.pose: position and orientation of base_link in odom frame
- twist.twist: linear and angular velocities in base_link frame

This node transforms the pose to express it in base_link coordinates while
keeping the child_frame_id as camera_link for visualization purposes.

Uses robust quaternion and rotation matrix conversions (Shepperd's method).
Transform from: ros2 run tf2_ros tf2_echo odom base_link
"""

from math import sqrt, atan2, asin
from typing import Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import rclpy.time

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
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(q: QuaternionTuple, vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Rotate a 3D vector using a quaternion (Hamilton product method)."""
    qx, qy, qz, qw = _normalize_quaternion(q)
    vx, vy, vz = vec

    # v' = q * v * q^(-1) using optimized formula
    uvx = 2.0 * (qy * vz - qz * vy)
    uvy = 2.0 * (qz * vx - qx * vz)
    uvz = 2.0 * (qx * vy - qy * vx)

    uuvx = 2.0 * (qy * uvz - qz * uvy)
    uuvy = 2.0 * (qz * uvx - qx * uvz)
    uuvz = 2.0 * (qx * uvy - qy * uvx)

    return (
        vx + qw * uvx + uuvx,
        vy + qw * uvy + uuvy,
        vz + qw * uvz + uuvz,
    )


def _matrix_multiply_vector(m: Tuple[Tuple[float, float, float], ...], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Multiply a 3x3 matrix by a 3D vector."""
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _matrix_multiply_matrix(a: Tuple[Tuple[float, float, float], ...], b: Tuple[Tuple[float, float, float], ...]) -> Tuple[Tuple[float, float, float], ...]:
    """Multiply two 3x3 rotation matrices."""
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )


def _matrix_transpose(m: Tuple[Tuple[float, float, float], ...]) -> Tuple[Tuple[float, float, float], ...]:
    """Transpose a 3x3 matrix (for rotation matrices, transpose = inverse)."""
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def _matrix_to_rpy(m: Tuple[Tuple[float, float, float], ...]) -> Tuple[float, float, float]:
    """
    Extract Roll-Pitch-Yaw from rotation matrix (ZYX convention).
    Returns (roll, pitch, yaw) in radians.
    """
    # Check for gimbal lock
    sin_pitch = -m[2][0]

    if abs(sin_pitch) >= 0.99999:  # Gimbal lock
        # Set roll to 0 and compute yaw
        roll = 0.0
        pitch = atan2(sin_pitch, 0.0)  # +/- pi/2
        yaw = atan2(-m[0][1], m[1][1])
    else:
        roll = atan2(m[2][1], m[2][2])
        pitch = asin(-m[2][0])
        yaw = atan2(m[1][0], m[0][0])

    return (roll, pitch, yaw)


def _quaternion_to_matrix(q: QuaternionTuple) -> Tuple[Tuple[float, float, float], ...]:
    """Convert quaternion to 3x3 rotation matrix (robust implementation)."""
    x, y, z, w = _normalize_quaternion(q)

    # Precompute repeated terms
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
        (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
        (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)),
    )


def _matrix_to_quaternion(m: Tuple[Tuple[float, float, float], ...]) -> QuaternionTuple:
    """
    Convert 3x3 rotation matrix to quaternion (Shepperd's method).
    This is the most numerically stable method, avoiding division by small numbers.
    """
    trace = m[0][0] + m[1][1] + m[2][2]

    if trace > 0.0:
        # w is the largest component
        s = sqrt(trace + 1.0) * 2.0  # s = 4 * w
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif (m[0][0] > m[1][1]) and (m[0][0] > m[2][2]):
        # x is the largest component
        s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0  # s = 4 * x
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        # y is the largest component
        s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0  # s = 4 * y
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        # z is the largest component
        s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0  # s = 4 * z
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    return _normalize_quaternion((x, y, z, w))


class OdomCameraRepublisher(Node):
    """
    Transform odometry from odom frame to base_link frame using real-time TF.

    Subscribes to /odom (pose in odom frame) and publishes /odom/camera (pose in base_link frame).
    Uses TF2 to look up the odom→base_link transform dynamically for each message.
    """

    def __init__(self) -> None:
        super().__init__('odom_camera_republisher')

        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/odom/camera')
        self.declare_parameter('use_tf_lookup', True)  # Use real-time TF lookup

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._use_tf_lookup = self.get_parameter('use_tf_lookup').get_parameter_value().bool_value

        # TF2 setup for real-time transform lookup
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._publisher = self.create_publisher(Odometry, self._output_topic, qos)
        self._subscription = self.create_subscription(Odometry, self._input_topic, self._handle_odom, qos)

        self._message_count = 0
        self._last_odom_msg = None

        # Create timer for logging every 5 seconds
        self._log_timer = self.create_timer(5.0, self._log_odometry)

        self.get_logger().info(f"Odometry Republisher started:")
        self.get_logger().info(f"  Input: {self._input_topic}")
        self.get_logger().info(f"  Output: {self._output_topic}")
        self.get_logger().info(f"  Logging every 5 seconds")

    def _handle_odom(self, msg: Odometry) -> None:
        """
        Transform odometry from odom frame to base_link frame using rotation matrices.
        Simply rotates the coordinate frame - NO translation subtraction.
        """
        self._message_count += 1

        # Extract odom pose
        p_odom = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        q_odom = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

        # Convert odom quaternion to rotation matrix
        R_odom = _quaternion_to_matrix(tuple(q_odom))

        # For robot-centric odometry, we want position in the robot's frame
        # p_base = R_odom^T * p_odom (rotate by inverse of robot's orientation)
        R_odom_inv = _matrix_transpose(R_odom)
        p_base = _matrix_multiply_vector(R_odom_inv, tuple(p_odom))

        # For orientation: apply the static 90-degree frame rotation
        # This aligns the odom frame axes with base_link axes
        q_static = [0.0, 0.0, -0.7071, 0.7071]  # 90 deg rotation around Z
        q_static_inv = _quaternion_conjugate(tuple(q_static))

        # Transform orientation: q_result = q_static_inv * q_odom
        q_result = _quaternion_multiply(q_static_inv, tuple(q_odom))

        # Extract RPY for logging
        R_result = _quaternion_to_matrix(q_result)
        roll, pitch, yaw = _matrix_to_rpy(R_result)

        # Transform velocities to robot frame
        v_linear = _matrix_multiply_vector(R_odom_inv, (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ))
        v_angular = _matrix_multiply_vector(R_odom_inv, (
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ))

        # Create output message
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.header.frame_id = 'base_link'
        new_msg.child_frame_id = 'camera_link'

        new_msg.pose.pose.position.x = p_base[0]
        new_msg.pose.pose.position.y = p_base[1]
        new_msg.pose.pose.position.z = p_base[2]
        new_msg.pose.pose.orientation.x = q_result[0]
        new_msg.pose.pose.orientation.y = q_result[1]
        new_msg.pose.pose.orientation.z = q_result[2]
        new_msg.pose.pose.orientation.w = q_result[3]

        new_msg.twist.twist.linear.x = v_linear[0]
        new_msg.twist.twist.linear.y = v_linear[1]
        new_msg.twist.twist.linear.z = v_linear[2]
        new_msg.twist.twist.angular.x = v_angular[0]
        new_msg.twist.twist.angular.y = v_angular[1]
        new_msg.twist.twist.angular.z = v_angular[2]

        new_msg.pose.covariance = msg.pose.covariance
        new_msg.twist.covariance = msg.twist.covariance

        # Store last message for timer-based logging
        self._last_odom_msg = {
            'p_odom': p_odom,
            'q_odom': q_odom,
            'p_base': p_base,
            'q_result': q_result,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'v_linear': v_linear,
            'v_angular': v_angular,
            'R_odom': R_odom,
        }

        self._publisher.publish(new_msg)

    def _log_odometry(self):
        """Log odometry data every 2 seconds (timer callback)"""
        if self._last_odom_msg is None:
            return

        msg = self._last_odom_msg
        p_odom = msg['p_odom']
        q_odom = msg['q_odom']
        p_base = msg['p_base']
        q_result = msg['q_result']
        roll = msg['roll']
        pitch = msg['pitch']
        yaw = msg['yaw']
        v_linear = msg['v_linear']
        v_angular = msg['v_angular']
        R_odom = msg['R_odom']

        roll_odom, pitch_odom, yaw_odom = _matrix_to_rpy(R_odom)

        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Odometry Status (every 5s):")
        self.get_logger().info(f"Input /odom (world-fixed frame):")
        self.get_logger().info(f"  pos  = [{p_odom[0]:8.4f}, {p_odom[1]:8.4f}, {p_odom[2]:8.4f}] m")
        self.get_logger().info(f"  quat = [{q_odom[0]:7.4f}, {q_odom[1]:7.4f}, {q_odom[2]:7.4f}, {q_odom[3]:7.4f}]")
        self.get_logger().info(f"  RPY  = [{roll_odom*57.3:7.2f}, {pitch_odom*57.3:7.2f}, {yaw_odom*57.3:7.2f}] deg")
        self.get_logger().info(f"Output /odom/camera (robot-centric frame):")
        self.get_logger().info(f"  pos  = [{p_base[0]:8.4f}, {p_base[1]:8.4f}, {p_base[2]:8.4f}] m")
        self.get_logger().info(f"  quat = [{q_result[0]:7.4f}, {q_result[1]:7.4f}, {q_result[2]:7.4f}, {q_result[3]:7.4f}]")
        self.get_logger().info(f"  RPY  = [{roll*57.3:7.2f}, {pitch*57.3:7.2f}, {yaw*57.3:7.2f}] deg")
        self.get_logger().info(f"Linear velocity:")
        self.get_logger().info(f"  v = [{v_linear[0]:7.4f}, {v_linear[1]:7.4f}, {v_linear[2]:7.4f}] m/s")
        self.get_logger().info(f"Angular velocity:")
        self.get_logger().info(f"  ω = [{v_angular[0]:7.4f}, {v_angular[1]:7.4f}, {v_angular[2]:7.4f}] rad/s")
        self.get_logger().info("=" * 70)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomCameraRepublisher()
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
