#!/usr/bin/env python3
"""Minimal motion controller for early robot bringup."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class RobotController(Node):
    """Publishes a simple forward-then-turn patrol pattern to /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("robot_controller")

        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("forward_duration", 3.0)
        self.declare_parameter("turn_duration", 1.6)
        self.declare_parameter("publish_hz", 10.0)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.forward_duration = float(self.get_parameter("forward_duration").value)
        self.turn_duration = float(self.get_parameter("turn_duration").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        if self.publish_hz <= 0.0:
            self.publish_hz = 10.0

        self.cycle_duration = self.forward_duration + self.turn_duration
        self.start_time_sec = self.get_clock().now().nanoseconds / 1e9

        self.cmd_pub = self.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self._tick)

        self.get_logger().info(
            "robot_controller started with linear_speed=%.2f, angular_speed=%.2f, "
            "forward_duration=%.2f, turn_duration=%.2f"
            % (
                self.linear_speed,
                self.angular_speed,
                self.forward_duration,
                self.turn_duration,
            )
        )

    def _tick(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self.start_time_sec
        phase = elapsed % self.cycle_duration if self.cycle_duration > 0.0 else 0.0

        msg = Twist()
        if phase < self.forward_duration:
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed

        self.cmd_pub.publish(msg)

    def stop_robot(self) -> None:
        stop = Twist()
        self.cmd_pub.publish(stop)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
