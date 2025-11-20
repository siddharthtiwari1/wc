#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.twist_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel_out',
            10
        )
        
        self.get_logger().info('Twist to TwistStamped converter started')
    
    def twist_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = ""  # diff_drive_controller doesn't need frame_id
        twist_stamped.twist = msg
        
        self.publisher.publish(twist_stamped)
        
        # Debug output
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.get_logger().info(f'Publishing: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()