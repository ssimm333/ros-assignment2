#!/usr/bin/env python3
"""Bridges Nav2 Twist commands to TwistStamped for the diff drive controller.

Nav2 publishes velocity commands as geometry_msgs/Twist on /cmd_vel,
but the ros2_control diff_drive_controller expects geometry_msgs/TwistStamped
on /diff_drive_controller/cmd_vel. This node sits in between and converts
one to the other by wrapping the Twist in a TwistStamped with a timestamp
and frame_id.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelay(Node):

    def __init__(self):
        super().__init__('twist_relay')
        # publish the stamped version to the diff drive controller
        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        # subscribe to the plain Twist that Nav2 outputs
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._cb, 10)

    def _cb(self, msg: Twist):
        # wrap the incoming Twist in a TwistStamped and forward it
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TwistRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
