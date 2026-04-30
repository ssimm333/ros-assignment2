import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse

from bt_interfaces.action import RotateX

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


import math
import time

class RotateXServer(Node):
    def __init__(self):
        super().__init__('rotate_x_server')
        self._action_server = ActionServer(self, 
                                        RotateX,
                                        'rotate_x',
                                        execute_callback=self.execute_callback,
                                        goal_callback=self.goal_callback,
                                        cancel_callback=self.cancel_callback,
                                        callback_group=ReentrantCallbackGroup())
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal = RotateX.Goal()
        self.current_yaw = 0.0

    
    def odom_callback(self, msg):
        self.current_yaw = math.atan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                                      msg.pose.pose.orientation.w * msg.pose.pose.orientation.w + msg.pose.pose.orientation.x * msg.pose.pose.orientation.x - msg.pose.pose.orientation.y * msg.pose.pose.orientation.y - msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request: Rotate to %.2f radians' % goal_request.angle)
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Rotate to %.2f radians' % goal_handle.request.angle)
        self.is_executing = True
        starting_yaw = self.current_yaw
        feedback_msg = RotateX.Feedback()
        tolerance = goal_handle.request.tolerance
        rotation_speed = goal_handle.request.rotation_speed
        target_angle = starting_yaw + goal_handle.request.angle
        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                self.is_executing = False
                stop_msg = Twist()
                self.cmd_pub.publish(stop_msg)
                result = RotateX.Result()
                result.success = False
                return result
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)
            feedback_msg.current_angle = self.current_yaw
            feedback_msg.angle_diff = angle_diff
            goal_handle.publish_feedback(feedback_msg)
            if abs(angle_diff) < tolerance:
                self.get_logger().info('Goal succeeded')
                goal_handle.succeed()
                self.is_executing = False
                stop_msg = Twist()
                self.cmd_pub.publish(stop_msg)
                result = RotateX.Result()
                result.success = True
                return result

            cmd_msg = Twist()
            if angle_diff > 0:
                cmd_msg.angular.z = rotation_speed 
            else:
                cmd_msg.angular.z = -rotation_speed
            self.cmd_pub.publish(cmd_msg)
            time.sleep(0.1)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
def main(args=None):
    rclpy.init(args=args)
    rotate_x_server = RotateXServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(rotate_x_server, executor=executor)
    rotate_x_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()