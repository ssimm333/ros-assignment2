#!/usr/bin/env python3
"""Battery simulator: drains over time, recharges when docked.

Publishes Bool on /battery_level_low at 1 Hz.
Subscribes to /is_docked (Bool) to trigger recharging.
Uses hysteresis (low_threshold / full_threshold) to prevent toggling.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class BatterySimulator(Node):

    def __init__(self):
        super().__init__('battery_simulator')

        # Parameters
        self.declare_parameter('drain_rate', 0.05)      # % per second
        self.declare_parameter('charge_rate', 2.0)       # % per second when docked
        self.declare_parameter('low_threshold', 20.0)    # % — triggers low warning
        self.declare_parameter('full_threshold', 90.0)   # % — clears low warning

        self.drain_rate = self.get_parameter('drain_rate').value
        self.charge_rate = self.get_parameter('charge_rate').value
        self.low_threshold = self.get_parameter('low_threshold').value
        self.full_threshold = self.get_parameter('full_threshold').value

        # State
        self.level = 100.0    # battery percentage
        self.is_docked = False
        self.is_low = False   # hysteresis flag

        # Publisher
        self.pub = self.create_publisher(Bool, '/battery_level_low', 10)

        # Subscriber
        self.create_subscription(Bool, '/is_docked', self._dock_cb, 10)

        # 1 Hz timer
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'Battery simulator started: drain={self.drain_rate}%/s, '
            f'charge={self.charge_rate}%/s, '
            f'low={self.low_threshold}%, full={self.full_threshold}%')

    def _dock_cb(self, msg: Bool):
        self.is_docked = msg.data

    def _tick(self):
        # Drain or charge
        if self.is_docked:
            self.level = min(100.0, self.level + self.charge_rate)
        else:
            self.level = max(0.0, self.level - self.drain_rate)

        # Hysteresis logic
        if not self.is_low and self.level <= self.low_threshold:
            self.is_low = True
            self.get_logger().warn(f'Battery LOW: {self.level:.1f}%')
        elif self.is_low and self.level >= self.full_threshold:
            self.is_low = False
            self.get_logger().info(f'Battery recharged: {self.level:.1f}%')

        # Publish
        msg = Bool()
        msg.data = self.is_low
        self.pub.publish(msg)

        # Periodic log every 10 seconds (level changes ~5% at default rate)
        if int(self.level * 10) % 100 == 0:
            state = 'CHARGING' if self.is_docked else 'DRAINING'
            self.get_logger().info(f'Battery: {self.level:.1f}% [{state}]')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatterySimulator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
