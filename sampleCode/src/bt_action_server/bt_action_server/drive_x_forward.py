import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse

from bt_interfaces.action import DriveX

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import time

class DriveXForwardServer(Node):
    def __init__(self):
        super().__init__('drive_x_forward_server')
        self._action_server = ActionServer(self, 
                                        DriveX,
                                        'drive_x',
                                        execute_callback=self.execute_callback,
                                        goal_callback=self.goal_callback,
                                        cancel_callback=self.cancel_callback,
                                        callback_group=ReentrantCallbackGroup())
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal = DriveX.Goal()
        self.current_position = (0.0, 0.0)

    
    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request: Drive forward %.2f meters' % goal_request.distance)
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Drive forward %.2f meters' % goal_handle.request.distance)
        self.is_executing = True
        tolerance = 0.05  # meters
        starting_position = self.current_position
        feedback_msg = DriveX.Feedback()
        tolerance = goal_handle.request.tolerance
        speed = goal_handle.request.speed
        target_distance = goal_handle.request.distance
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                self.is_executing = False
                stop_msg = Twist()
                self.cmd_pub.publish(stop_msg)
                result = DriveX.Result()
                result.success = False
                return result
            distance_travelled = math.sqrt((starting_position[0] - self.current_position[0])**2 + (starting_position[1] - self.current_position[1])**2)
            distance_diff = target_distance - distance_travelled
            feedback_msg.current_distance = distance_travelled
            feedback_msg.distance_diff = distance_diff
            goal_handle.publish_feedback(feedback_msg)
            if abs(distance_diff) < tolerance:
                self.get_logger().info('Goal succeeded')
                goal_handle.succeed()
                self.is_executing = False
                stop_msg = Twist()
                self.cmd_pub.publish(stop_msg)
                result = DriveX.Result()
                result.success = True
                return result

            cmd_msg = Twist()
            if distance_diff > 0:
                cmd_msg.linear.x = speed
            else:
                cmd_msg.linear.x = -speed
            self.cmd_pub.publish(cmd_msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    drive_x_forward_server = DriveXForwardServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(drive_x_forward_server, executor=executor)
    drive_x_forward_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()