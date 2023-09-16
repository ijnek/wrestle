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
from walk_interfaces.action import Crouch
from math import radians

class MotionManager(Node):

  def __init__(self):
    super().__init__('motion_manager')

    walk_cb_group = MutuallyExclusiveCallbackGroup()
    timer_cb_group = MutuallyExclusiveCallbackGroup()

    self.timer = self.create_timer(0.02, self.timer_callback, callback_group=timer_cb_group)
    self.walk_client = ActionClient(self, Walk, 'walk', callback_group=walk_cb_group)

    self.opponent_subscription = self.create_subscription(PointStamped, 'opponent_point',
                                                          self.opponent_callback, 10)

    self.crouch_action_client = ActionClient(self, Crouch, 'motion/crouch')

    self.last_twist = None

    self.last_time_opponent_detected = self.get_clock().now()
    self.opponent_heading_average = 0.0

    self.initial = True
    self.crouched = False
    self.spin = False

  def opponent_callback(self, opponent_point):
    # Decide on twist here

    heading = atan2(opponent_point.point.y, opponent_point.point.x)
    self.get_logger().info('Heading: %f' % heading)
    self.opponent_heading_average = self.opponent_heading_average * 0.7 + heading * 0.3
    # print("self.opponent_heading_average: ", self.opponent_heading_average)
    self.last_time_opponent_detected = Time.from_msg(opponent_point.header.stamp)

  def crouch_done_callback(self, _):
    self.crouched = True

  def timer_callback(self):

    if self.initial:
      self._send_goal_future = self.crouch_action_client.send_goal_async(Crouch.Goal())
      self._send_goal_future.add_done_callback(self.crouch_done_callback)
      self.initial = False
      return

    if not self.crouched:
      # In the process of crouching, don't do anything here
      return

    twist = Twist()
    time_elapsed_since_opponent_detected = \
      (self.get_clock().now() - self.last_time_opponent_detected).nanoseconds / 1e9

    if time_elapsed_since_opponent_detected > 2.0:
      self.spin = True
    elif not self.spin and abs(self.opponent_heading_average) > radians(20):
      # print("spin!")
      self.spin = True
    elif self.spin and abs(self.opponent_heading_average) < radians(10):
      # print("not spin!")
      self.spin = False

    if self.spin:
      twist.angular.z = 1.0 if self.opponent_heading_average > 0 else -1.0
    else:
      twist.linear.x = 0.4
      twist.angular.z = self.opponent_heading_average * 0.5

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
