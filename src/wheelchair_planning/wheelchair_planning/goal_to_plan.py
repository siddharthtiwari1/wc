#!/usr/bin/env python3

"""
Goal to Plan Bridge
===================
Bridges RViz "2D Goal Pose" to Nav2 planner action.
Subscribes to /goal_pose, calls /compute_path_to_pose action, publishes /plan.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer, TransformListener


class GoalToPlan(Node):
    def __init__(self):
        super().__init__('goal_to_plan')

        # TF for getting robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client to planner
        self.planner_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose'
        )

        # Subscribe to goal from RViz
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        # Publish planned path
        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.get_logger().info('Goal to Plan bridge ready')
        self.get_logger().info('  Listening on: /goal_pose')
        self.get_logger().info('  Publishing to: /plan')

    def goal_callback(self, goal_msg: PoseStamped):
        self.get_logger().info(
            f'Goal received: ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})'
        )

        # Wait for action server
        if not self.planner_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Planner action server not available!')
            return

        # Create goal
        goal = ComputePathToPose.Goal()
        goal.goal = goal_msg
        goal.planner_id = 'GridBased'
        goal.use_start = False  # Use robot's current pose as start

        # Send goal
        self.get_logger().info('Sending goal to planner...')
        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by planner')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        path = result.path

        if len(path.poses) > 0:
            self.get_logger().info(f'Path received with {len(path.poses)} waypoints')
            self.path_pub.publish(path)
        else:
            self.get_logger().warn('Empty path received - no valid path found')


def main(args=None):
    rclpy.init(args=args)
    node = GoalToPlan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
