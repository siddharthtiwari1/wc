#!/usr/bin/env python3

"""Republish IMU data into the base_link frame using a fixed rotation."""

from math import sqrt
from typing import Iterable, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

QuaternionTuple = Tuple[float, float, float, float]


class ImuBaseLinkRepublisher(Node):
    """Convert IMU readings from the camera optical frame into base_link."""

    def __init__(self) -> None:
        super().__init__('imu_base_link_republisher')
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('base_to_optical_quaternion', [-0.5, 0.5, -0.5, 0.5])

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._output_frame = self.get_parameter('output_frame').get_parameter_value().string_value

        rotation_param = self._get_quaternion_param('base_to_optical_quaternion', [-0.5, 0.5, -0.5, 0.5])
        self._base_to_optical = rotation_param
        self._optical_to_base = self._quaternion_conjugate(rotation_param)

        self._sensor_to_base_matrix = self._quaternion_to_matrix(self._optical_to_base)

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._publisher = self.create_publisher(Imu, self._output_topic, qos)
        self._subscription = self.create_subscription(Imu, self._input_topic, self._handle_imu, qos)
        self._logged_first_message = False

        self.get_logger().info(
            'Republishing %s -> %s using base_to_optical quaternion %s (output frame %s).',
            self._input_topic,
            self._output_topic,
            self._base_to_optical,
            self._output_frame,
        )

    def _get_quaternion_param(self, name: str, default: Iterable[float]) -> QuaternionTuple:
        value = self.get_parameter(name).get_parameter_value().double_array_value
        if len(value) != 4:
            self.get_logger().warn(
                '%s must contain 4 elements; defaulting to %s.', name, list(default)
            )
            value = list(default)
        return self._normalize_quaternion(tuple(value))

    def _handle_imu(self, msg: Imu) -> None:
        republished = Imu()
        republished.header = msg.header
        republished.header.frame_id = self._output_frame

        republished.orientation = self._transform_orientation(msg.orientation)
        republished.angular_velocity = self._rotate_vector(msg.angular_velocity)
        republished.linear_acceleration = self._rotate_vector(msg.linear_acceleration)

        republished.orientation_covariance = msg.orientation_covariance
        republished.angular_velocity_covariance = msg.angular_velocity_covariance
        republished.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self._publisher.publish(republished)
        if not self._logged_first_message:
            self._logged_first_message = True
            self.get_logger().info('First IMU message republished with base_link alignment.')

    def _transform_orientation(self, orientation: Quaternion) -> Quaternion:
        quat = (orientation.x, orientation.y, orientation.z, orientation.w)
        if not self._is_valid_quaternion(quat):
            quat = (0.0, 0.0, 0.0, 1.0)

        rotation_sensor_to_world = self._quaternion_to_matrix(quat)
        rotation_base_to_world = self._matrix_multiply(rotation_sensor_to_world, self._sensor_to_base_matrix)
        rotated_quat = self._matrix_to_quaternion(rotation_base_to_world)

        output = Quaternion()
        output.x, output.y, output.z, output.w = rotated_quat
        return output

    def _rotate_vector(self, vector: Vector3) -> Vector3:
        vx = (
            self._sensor_to_base_matrix[0][0] * vector.x
            + self._sensor_to_base_matrix[0][1] * vector.y
            + self._sensor_to_base_matrix[0][2] * vector.z
        )
        vy = (
            self._sensor_to_base_matrix[1][0] * vector.x
            + self._sensor_to_base_matrix[1][1] * vector.y
            + self._sensor_to_base_matrix[1][2] * vector.z
        )
        vz = (
            self._sensor_to_base_matrix[2][0] * vector.x
            + self._sensor_to_base_matrix[2][1] * vector.y
            + self._sensor_to_base_matrix[2][2] * vector.z
        )

        output = Vector3()
        output.x, output.y, output.z = vx, vy, vz
        return output

    @staticmethod
    def _quaternion_conjugate(q: QuaternionTuple) -> QuaternionTuple:
        x, y, z, w = q
        return (-x, -y, -z, w)

    @staticmethod
    def _matrix_multiply(a, b):
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

    @staticmethod
    def _quaternion_to_matrix(q: QuaternionTuple):
        x, y, z, w = ImuBaseLinkRepublisher._normalize_quaternion(q)
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

    @staticmethod
    def _matrix_to_quaternion(m) -> QuaternionTuple:
        trace = m[0][0] + m[1][1] + m[2][2]
        if trace > 0.0:
            s = sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (m[2][1] - m[1][2]) / s
            y = (m[0][2] - m[2][0]) / s
            z = (m[1][0] - m[0][1]) / s
        elif (m[0][0] > m[1][1]) and (m[0][0] > m[2][2]):
            s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
            w = (m[2][1] - m[1][2]) / s
            x = 0.25 * s
            y = (m[0][1] + m[1][0]) / s
            z = (m[0][2] + m[2][0]) / s
        elif m[1][1] > m[2][2]:
            s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
            w = (m[0][2] - m[2][0]) / s
            x = (m[0][1] + m[1][0]) / s
            y = 0.25 * s
            z = (m[1][2] + m[2][1]) / s
        else:
            s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
            w = (m[1][0] - m[0][1]) / s
            x = (m[0][2] + m[2][0]) / s
            y = (m[1][2] + m[2][1]) / s
            z = 0.25 * s
        return ImuBaseLinkRepublisher._normalize_quaternion((x, y, z, w))

    @staticmethod
    def _normalize_quaternion(q: QuaternionTuple) -> QuaternionTuple:
        x, y, z, w = q
        norm = sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return 0.0, 0.0, 0.0, 1.0
        return x / norm, y / norm, z / norm, w / norm

    @staticmethod
    def _is_valid_quaternion(q: QuaternionTuple) -> bool:
        return not (q[0] == 0.0 and q[1] == 0.0 and q[2] == 0.0 and q[3] == 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuBaseLinkRepublisher()
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
