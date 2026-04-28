"""Simulates battery drain and recharge for the low-battery behavior task."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class BatterySimulator(Node):
    def __init__(self):
        super().__init__("battery_simulator")

        # Parameters
        self.declare_parameter("drain_rate", 0.5)       # % per second
        self.declare_parameter("recharge_rate", 5.0)     # % per second
        self.declare_parameter("low_threshold", 20.0)    # % to trigger low
        self.declare_parameter("full_threshold", 90.0)   # % to clear low

        self.drain_rate = self.get_parameter("drain_rate").value
        self.recharge_rate = self.get_parameter("recharge_rate").value
        self.low_threshold = self.get_parameter("low_threshold").value
        self.full_threshold = self.get_parameter("full_threshold").value

        self.level = 100.0
        self.is_docked = False

        self.pub = self.create_publisher(Bool, "battery_level_low", 10)
        self.dock_sub = self.create_subscription(Bool, "is_docked", self.dock_cb, 10)

        # Tick at 1 Hz
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            f"Battery simulator started: drain={self.drain_rate}%/s, "
            f"low_threshold={self.low_threshold}%"
        )

    def dock_cb(self, msg: Bool):
        self.is_docked = msg.data

    def tick(self):
        if self.is_docked:
            self.level = min(100.0, self.level + self.recharge_rate)
        else:
            self.level = max(0.0, self.level - self.drain_rate)

        is_low = self.level < self.low_threshold
        # Hysteresis: once low, stay low until recharged above full_threshold
        if not is_low and self.level < self.full_threshold:
            # Check if we were previously low — keep reporting low
            pass

        msg = Bool()
        msg.data = self.level < self.low_threshold
        self.pub.publish(msg)

        self.get_logger().debug(
            f"Battery: {self.level:.1f}% | docked={self.is_docked} | low={msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
