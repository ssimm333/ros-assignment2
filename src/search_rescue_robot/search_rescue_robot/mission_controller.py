#!/usr/bin/env python3
"""Mission controller implementing assignment tasks with a behavior tree."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from search_rescue_robot.behavior_tree import (
    Action,
    Blackboard,
    Condition,
    Fallback,
    Sequence,
    Status,
    TimeoutDecorator,
)


@dataclass(frozen=True)
class XY:
    x: float
    y: float


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


WORLD_POINTS: Dict[str, XY] = {
    "survivor": XY(15.1, 13.4),
    "dam": XY(8.7, -11.6),
    "medical_kit": XY(-6.3, -16.9),
    "fire": XY(-14.2, 10.8),
    "exit": XY(2.9, 17.2),
    "dock": XY(24.89, 0.0),
}


class WaypointNavigator:
    """Direct velocity-based waypoint navigation for the diff-drive controller."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
        self.target: Optional[Tuple[float, float, float]] = None
        self.current_xy = XY(0.0, 0.0)
        self.current_yaw = 0.0
        self.position_tolerance = 0.35
        self.max_linear = 0.25
        self.max_angular = 0.8
        self.k_linear = 0.35
        self.k_angular = 1.8

    def update_pose(self, x: float, y: float, yaw: float) -> None:
        self.current_xy = XY(x, y)
        self.current_yaw = yaw

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        self.target = (x, y, yaw)
        return True

    def cancel(self) -> None:
        self.target = None
        self._publish_stop()

    def status(self) -> Status:
        if self.target is None:
            return Status.FAILURE

        goal_x, goal_y, goal_yaw = self.target
        dx = goal_x - self.current_xy.x
        dy = goal_y - self.current_xy.y
        distance = math.hypot(dx, dy)

        if distance <= self.position_tolerance:
            yaw_error = _normalize_angle(goal_yaw - self.current_yaw)
            if abs(yaw_error) <= 0.15:
                self._publish_stop()
                return Status.SUCCESS

            twist = Twist()
            twist.angular.z = max(-self.max_angular, min(self.max_angular, 1.5 * yaw_error))
            self.cmd_pub.publish(twist)
            return Status.RUNNING

        heading = math.atan2(dy, dx)
        yaw_error = _normalize_angle(heading - self.current_yaw)

        twist = Twist()
        linear = min(self.max_linear, self.k_linear * distance)
        if abs(yaw_error) > 0.75:
            linear *= 0.25
        twist.linear.x = max(0.0, linear)
        twist.angular.z = max(-self.max_angular, min(self.max_angular, self.k_angular * yaw_error))
        self.cmd_pub.publish(twist)
        return Status.RUNNING

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


class MissionController(Node):
    """Coordinates full mission tasks using a behavior tree."""

    def __init__(self) -> None:
        super().__init__("mission_controller")

        self.blackboard = Blackboard()
        self.navigator = WaypointNavigator(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.event_pub = self.create_publisher(String, "/mission/events", 10)
        self.camera_pub = self.create_publisher(Float64MultiArray, "/camera_pan_controller/commands", 10)
        self.battery_sub = self.create_subscription(Bool, "/battery_level_low", self._battery_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        self.status_srv = self.create_service(Trigger, "~/get_mission_status", self._status_srv_cb)

        self.battery_low = False
        self._active_target: Optional[Tuple[float, float, float]] = None
        self._wait_started_at: Optional[float] = None
        self._docking = False

        self.tree = self._build_tree()
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("mission_controller started")

    def _build_tree(self) -> Sequence:
        task_1 = Sequence(
            "task_1_medical_delivery",
            [
                self._nav_task("nav_survivor_standoff", self._survivor_standoff, timeout=140.0),
                Action("wait_1s", lambda: self._wait_action(1.0)),
                Action("publish_survivor_tf", self._publish_survivor_tf),
                self._nav_task("nav_medkit", lambda: WORLD_POINTS["medical_kit"], timeout=140.0),
                self._nav_task("return_survivor_standoff", self._survivor_standoff, timeout=140.0),
            ],
        )

        task_2 = Sequence(
            "task_2_scan_dam",
            [
                self._nav_task("nav_dam_standoff", self._dam_standoff, timeout=140.0),
                Action("camera_center", lambda: self._camera_pan(0.0)),
                Action("camera_left", lambda: self._camera_pan(math.radians(10.0))),
                Action("camera_right", lambda: self._camera_pan(math.radians(-10.0))),
            ],
        )

        task_5 = Sequence(
            "task_5_exit",
            [
                self._nav_task("nav_exit_touch", lambda: WORLD_POINTS["exit"], timeout=160.0),
                Action("mission_finish", self._finish_mission),
            ],
        )

        fire_safety = Fallback(
            "fire_safety_fallback",
            [
                Condition("safe_from_fire", self._is_safe_from_fire),
                self._nav_task("retreat_to_safe_zone", lambda: XY(0.0, 0.0), timeout=70.0),
            ],
        )

        return Sequence("root", [fire_safety, task_1, task_2, task_5])

    def _nav_task(self, name: str, target_fn, timeout: float):
        return TimeoutDecorator(name + "_timeout", Action(name, lambda: self._navigate_action(target_fn)), self._now, timeout)

    def _battery_cb(self, msg: Bool) -> None:
        self.battery_low = msg.data

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.navigator.update_pose(x, y, yaw)

    def _status_srv_cb(self, request, response):
        del request
        response.success = self.blackboard.mission_complete
        response.message = "complete" if self.blackboard.mission_complete else "in_progress"
        return response

    def _tick(self) -> None:
        if self.blackboard.mission_complete:
            self.navigator.cancel()
            return

        if self.battery_low:
            self._handle_low_battery()
            return

        if self._docking:
            self._docking = False
            self._emit_event("battery_normal_resume")

        status = self.tree.tick()
        if status == Status.FAILURE:
            self._emit_event("mission_tree_failed")

    def _handle_low_battery(self) -> None:
        if not self._docking:
            self._docking = True
            self._emit_event("battery_low_docking")
            self._active_target = None
            self.navigator.cancel()

        dock = WORLD_POINTS["dock"]
        self._navigate_to(dock.x, dock.y, 0.0)

    def _navigate_action(self, target_fn) -> Status:
        target = target_fn()
        return self._navigate_to(target.x, target.y, 0.0)

    def _navigate_to(self, x: float, y: float, yaw: float) -> Status:
        desired = (x, y, yaw)

        if self._active_target != desired:
            self.navigator.cancel()
            self.navigator.send_goal(x, y, yaw)
            self._active_target = desired
            self._emit_event(f"nav_goal_sent:{x:.2f},{y:.2f}")
            return Status.RUNNING

        status = self.navigator.status()
        if status == Status.SUCCESS:
            self._active_target = None
        elif status == Status.FAILURE:
            self._active_target = None
        return status

    def _wait_action(self, seconds: float) -> Status:
        if self._wait_started_at is None:
            self._wait_started_at = self._now()
            return Status.RUNNING

        if self._now() - self._wait_started_at >= seconds:
            self._wait_started_at = None
            return Status.SUCCESS
        return Status.RUNNING

    def _publish_survivor_tf(self) -> Status:
        survivor = WORLD_POINTS["survivor"]
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "survivor_marker"
        tf.transform.translation.x = survivor.x
        tf.transform.translation.y = survivor.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)
        self._emit_event("survivor_tf_published")
        return Status.SUCCESS

    def _camera_pan(self, angle_rad: float) -> Status:
        cmd = Float64MultiArray()
        cmd.data = [angle_rad]
        self.camera_pub.publish(cmd)
        return self._wait_action(1.0)

    def _finish_mission(self) -> Status:
        self.blackboard.mission_complete = True
        self.navigator.cancel()
        self._emit_event("mission_complete")
        return Status.SUCCESS

    def _is_safe_from_fire(self) -> bool:
        fire = WORLD_POINTS["fire"]
        d = math.hypot(self.navigator.current_xy.x - fire.x, self.navigator.current_xy.y - fire.y)
        return d >= 3.0

    def _survivor_standoff(self) -> XY:
        return self._standoff_point(WORLD_POINTS["survivor"], 1.0)

    def _dam_standoff(self) -> XY:
        return self._standoff_point(WORLD_POINTS["dam"], 0.5)

    def _standoff_point(self, target: XY, distance: float) -> XY:
        # Keep the waypoint fixed relative to the spawn origin so it does not drift with the robot.
        origin = XY(0.0, 0.0)
        vx = target.x - origin.x
        vy = target.y - origin.y
        n = math.hypot(vx, vy)
        if n < 1e-3:
            return XY(target.x - distance, target.y)
        ux = vx / n
        uy = vy / n
        return XY(target.x - ux * distance, target.y - uy * distance)

    def _emit_event(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.blackboard.last_event = text
        self.event_pub.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
