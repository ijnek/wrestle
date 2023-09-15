#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from walk_interfaces.action import Walk
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, PointStamped
from math import atan2, sin, cos


class MotionManager(Node):

  def __init__(self):
    super().__init__('motion_manager')

    walk_cb_group = MutuallyExclusiveCallbackGroup()
    timer_cb_group = MutuallyExclusiveCallbackGroup()

    self.timer = self.create_timer(0.02, self.timer_callback, callback_group=timer_cb_group)
    self.walk_client = ActionClient(self, Walk, 'walk', callback_group=walk_cb_group)

    self.opponent_subscription = self.create_subscription(PointStamped, 'opponent_point',
                                                          self.opponent_callback, 10)

    self.last_twist = None

    self.last_time_opponent_detected = self.get_clock().now()
    self.last_heading = 0.0

  def opponent_callback(self, opponent_point):
    # Decide on twist here

    heading = atan2(opponent_point.point.y, opponent_point.point.x)
    self.last_heading = heading
    self.get_logger().info('Heading: %f' % heading)
    twist = Twist()

    speed = 0.3
    twist.linear.x = speed * cos(heading)
    twist.linear.y = speed * sin(heading)
    twist.angular.z = heading * 0.3

    walk_goal = Walk.Goal()
    walk_goal.twist = twist
    self.walk_client.send_goal_async(walk_goal)
    self.last_twist = twist

    self.last_time_opponent_detected = Time.from_msg(opponent_point.header.stamp)

  def timer_callback(self):

    time_elapsed_since_opponent_detected = \
      (self.get_clock().now() - self.last_time_opponent_detected).nanoseconds / 1e9

    if time_elapsed_since_opponent_detected > 2.0:
      # Spin!
      twist = Twist()
      if self.last_heading > 0:
        twist.angular.z = 1.0
      else:
        twist.angular.z = -1.0

      if twist != self.last_twist:
        walk_goal = Walk.Goal()
        walk_goal.twist = twist
        self.walk_client.send_goal_async(walk_goal)
        self.last_twist = twist

def main(args=None):
  rclpy.init(args=args)
  motion_manager = MotionManager()
  executor = MultiThreadedExecutor()
  executor.add_node(motion_manager)
  executor.spin()
  motion_manager.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
