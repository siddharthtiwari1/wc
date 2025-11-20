#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np

def quat_to_mat(q):
    M = quaternion_matrix([q[0], q[1], q[2], q[3]])
    return M[:3, :3]

def mat_to_quat(R):
    M = np.eye(4); M[:3, :3] = R
    qx, qy, qz, qw = quaternion_from_matrix(M)
    return (qx, qy, qz, qw)

class CameraOdomRepublisher(Node):
    def __init__(self):
        super().__init__('camera_odom_republisher')
        # params
        self.declare_parameter('input_odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/camera/imu')
        self.declare_parameter('output_odom_topic', '/camera_odom')
        self.declare_parameter('target_child_frame', 'base_link')
        self.declare_parameter('use_imu_timestamp', True)
        self.declare_parameter('imu_time_tolerance_sec', 0.01)

        p = self.get_parameter
        self.in_odom  = p('input_odom_topic').value
        self.imu_topic= p('imu_topic').value
        self.out_odom = p('output_odom_topic').value
        self.child    = p('target_child_frame').value
        self.use_imu  = p('use_imu_timestamp').value
        self.tol      = p('imu_time_tolerance_sec').value

        self.tf_buf = Buffer()
        self.tf_ls  = TransformListener(self.tf_buf, self)

        self.sub_o  = self.create_subscription(Odometry, self.in_odom, self.odom_cb, 30)
        self.sub_i  = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 100)
        self.pub    = self.create_publisher(Odometry, self.out_odom, 30)

        self.latest_imu_stamp = None
        self.get_logger().info(f"In: {self.in_odom}, IMU: {self.imu_topic}, Out: {self.out_odom}, child→{self.child}")

    def imu_cb(self, msg: Imu):
        self.latest_imu_stamp = msg.header.stamp

    def odom_cb(self, msg: Odometry):
        parent = msg.header.frame_id
        cam_child = msg.child_frame_id or 'camera_link'

        # lookup transform: camera child -> target child (base_link)
        try:
            tf = self.tf_buf.lookup_transform(self.child, cam_child, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF {cam_child}->{self.child} not ready: {e}")
            return

        R_cb = quat_to_mat((tf.transform.rotation.x, tf.transform.rotation.y,
                            tf.transform.rotation.z, tf.transform.rotation.w))
        t_cb = np.array([tf.transform.translation.x,
                         tf.transform.translation.y,
                         tf.transform.translation.z], dtype=float)

        # input pose parent->cam_child
        p_pc = np.array([msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         msg.pose.pose.position.z], dtype=float)
        q_pc = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        R_pc = quat_to_mat(q_pc)

        # compose parent->base_link
        p_pb = p_pc + R_pc @ t_cb
        R_pb = R_pc @ R_cb
        q_pb = mat_to_quat(R_pb)

        # twists: rotate into base_link axes
        v_cam = np.array([msg.twist.twist.linear.x,
                          msg.twist.twist.linear.y,
                          msg.twist.twist.linear.z], dtype=float)
        w_cam = np.array([msg.twist.twist.angular.x,
                          msg.twist.twist.angular.y,
                          msg.twist.twist.angular.z], dtype=float)
        v_base = R_cb @ v_cam
        w_base = R_cb @ w_cam

        out = Odometry()
        out.header.frame_id = parent             # keep camera’s world
        out.child_frame_id  = self.child         # express body as base_link
        out.header.stamp    = msg.header.stamp   # may restamp to IMU below

        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = p_pb.tolist()
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = q_pb

        out.twist.twist.linear.x,  out.twist.twist.linear.y,  out.twist.twist.linear.z  = v_base.tolist()
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = w_base.tolist()

        # copy covariances (simple)
        out.pose.covariance  = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        # align timestamp to IMU if within tolerance
        if self.use_imu and self.latest_imu_stamp:
            dt = (self.latest_imu_stamp.sec + 1e-9*self.latest_imu_stamp.nanosec) - \
                 (out.header.stamp.sec  + 1e-9*out.header.stamp.nanosec)
            if abs(dt) <= self.tol:
                out.header.stamp = self.latest_imu_stamp

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(CameraOdomRepublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
