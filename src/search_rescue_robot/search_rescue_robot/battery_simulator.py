#!/usr/bin/env python3
"""Simulates battery drain and recharge for the rescue robot.

This node keeps track of a virtual battery level that drains while
the robot is driving around and charges when the robot is docked.
It publishes a Bool on /battery_level_low that the behavior tree
nodes (CheckBatteryOK, WaitForBatteryOK) subscribe to.

The docking status comes from /is_docked, which is published by
the PublishDockStatus BT node when the tree docks or undocks.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class BatterySimulator(Node):

    def __init__(self):
        super().__init__('battery_simulator')

        # declare ROS parameters so they can be tuned from launch files if needed
        self.declare_parameter('drain_rate', 0.05)          # % per second while driving
        self.declare_parameter('charge_rate', 2.0)          # % per second when docked
        self.declare_parameter('low_threshold', 20.0)       # battery % that triggers "low"
        self.declare_parameter('full_threshold', 90.0)      # battery % that clears "low"

        # read the parameter values into member variables
        self.drain_rate = self.get_parameter('drain_rate').value
        self.charge_rate = self.get_parameter('charge_rate').value
        self.low_threshold = self.get_parameter('low_threshold').value
        self.full_threshold = self.get_parameter('full_threshold').value

        # battery state
        self.level = 100.0    # start at full charge
        self.is_docked = False
        self.is_low = False   # hysteresis flag (see below)

        # publish whether battery is low on this topic
        self.pub = self.create_publisher(Bool, '/battery_level_low', 10)

        # listen to the BT node that says whether we are docked or not
        self.create_subscription(Bool, '/is_docked', self._dock_cb, 10)

        # run the simulation at 1 Hz (once per second)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'Battery simulator started: drain={self.drain_rate}%/s, '
            f'charge={self.charge_rate}%/s, '
            f'low={self.low_threshold}%, full={self.full_threshold}%')

    def _dock_cb(self, msg: Bool):
        # called whenever the BT publishes a dock status update
        self.is_docked = msg.data

    def _tick(self):
        # update the battery level based on whether we are docked or not
        if self.is_docked:
            # charging: increase level, cap at 100%
            self.level = min(100.0, self.level + self.charge_rate)
        else:
            # draining: decrease level, floor at 0%
            self.level = max(0.0, self.level - self.drain_rate)

        # hysteresis logic to avoid flickering between low and ok.
        # we only set is_low=True when we drop below 20% (low_threshold),
        # and only clear it when we charge back above 90% (full_threshold).
        # this prevents the robot from undocking at 21% and immediately
        # getting sent back to dock again.
        if not self.is_low and self.level <= self.low_threshold:
            self.is_low = True
            self.get_logger().warn(f'Battery LOW: {self.level:.1f}%')
        elif self.is_low and self.level >= self.full_threshold:
            self.is_low = False
            self.get_logger().info(f'Battery recharged: {self.level:.1f}%')

        # publish the current low status every tick so the BT can react
        msg = Bool()
        msg.data = self.is_low
        self.pub.publish(msg)

        # log the battery level roughly every 10 seconds for debugging
        if int(self.level * 10) % 100 == 0:
            state = 'CHARGING' if self.is_docked else 'DRAINING'
            self.get_logger().info(f'Battery: {self.level:.1f}% [{state}]')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatterySimulator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
