#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, quaternion_from_matrix, quaternion_inverse, quaternion_multiply
import numpy as np

def quat_to_mat(q):
    M = quaternion_matrix([q[0], q[1], q[2], q[3]])
    return M[:3, :3]

def mat_to_quat(R):
    M = np.eye(4); M[:3, :3] = R
    x, y, z, w = quaternion_from_matrix(M)
    return (x, y, z, w)

def rot_cov(C, R):
    """Rotate a 3x3 covariance with rotation matrix R (C' = R C R^T)."""
    C3 = np.array(C, dtype=float).reshape(3, 3)
    return (R @ C3 @ R.T).reshape(9).tolist()

class ImuToBaseRepublisher(Node):
    def __init__(self):
        super().__init__('imu_to_base_republisher')

        # Parameters
        self.declare_parameter('input_imu_topic', '/imu/data')
        self.declare_parameter('output_imu_topic', '/imu/odom_oriented')
        self.declare_parameter('target_frame', 'base_link')
        # If incoming IMU header.frame_id is empty or odd, you can override it:
        self.declare_parameter('source_frame_override', '')

        self.in_topic  = self.get_parameter('input_imu_topic').value
        self.out_topic = self.get_parameter('output_imu_topic').value
        self.target    = self.get_parameter('target_frame').value
        self.src_override = self.get_parameter('source_frame_override').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(Imu, self.in_topic, self.cb, 100)
        self.pub = self.create_publisher(Imu, self.out_topic, 100)

        self.get_logger().info(f"Re-expressing IMU {self.in_topic} into {self.target} → {self.out_topic}")

    def cb(self, msg: Imu):
        # Determine the IMU's source frame
        src_frame = self.src_override if self.src_override else (msg.header.frame_id or '')
        if not src_frame:
            self.get_logger().warn("IMU header.frame_id is empty and no source_frame_override provided; skipping.")
            return

        # Lookup TF: target_frame (base_link) ← source_frame (imu)
        try:
            tf = self.tf_buffer.lookup_transform(self.target, src_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF {src_frame} -> {self.target} not available: {e}")
            return

        # Build rotations
        q_bi = (tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w)  # imu -> base_link
        R_bi = quat_to_mat(q_bi)
        q_ib = quaternion_inverse([*q_bi])  # base_link -> imu

        # --- Orientation ---
        # Incoming orientation is typically q_wi (IMU-to-world). We want q_wb = q_wi ⊗ q_ib
        q_in = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        if all(np.isfinite(q_in)):
            q_wi = list(q_in)
            q_wb = quaternion_multiply(q_wi, q_ib)  # re-express in base_link
        else:
            # If orientation is unset (0s), propagate as is but set frame
            q_wb = q_in

        # --- Angular velocity & linear acceleration (vectors in IMU frame) ---
        # Re-express vectors in base_link: v_b = R_bi * v_i
        w_i = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=float)
        a_i = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=float)
        if np.all(np.isfinite(w_i)):
            w_b = R_bi @ w_i
        else:
            w_b = w_i
        if np.all(np.isfinite(a_i)):
            a_b = R_bi @ a_i
        else:
            a_b = a_i

        # --- Covariances (rotate 3x3 blocks) ---
        ori_cov = msg.orientation_covariance
        ang_cov = msg.angular_velocity_covariance
        lin_cov = msg.linear_acceleration_covariance
        if all(np.isfinite(ori_cov)):
            ori_cov = rot_cov(ori_cov, quat_to_mat(q_ib))  # orientation covariance transforms with q_ib (same as orientation)
        if all(np.isfinite(ang_cov)):
            ang_cov = rot_cov(ang_cov, R_bi)
        if all(np.isfinite(lin_cov)):
            lin_cov = rot_cov(lin_cov, R_bi)

        # --- Build output IMU ---
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target  # now expressed in base_link

        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_wb
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = w_b.tolist()
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = a_b.tolist()

        out.orientation_covariance = ori_cov
        out.angular_velocity_covariance = ang_cov
        out.linear_acceleration_covariance = lin_cov

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ImuToBaseRepublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
