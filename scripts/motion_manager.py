#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from walk_interfaces.action import Walk
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist


class MotionManager(Node):

  def __init__(self):
    super().__init__('motion_manager')

    walk_cb_group = MutuallyExclusiveCallbackGroup()
    timer_cb_group = MutuallyExclusiveCallbackGroup()

    self.timer = self.create_timer(0.02, self.timer_callback, callback_group=timer_cb_group)
    self.walk_client = ActionClient(self, Walk, 'walk', callback_group=walk_cb_group)

    self.start_time = self.get_clock().now()
    self.last_twist = None

  def timer_callback(self):
    # Decide on twist here
    twist = Twist()

    time_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
    if (time_elapsed < 10):
      # Move forward
      twist.linear.x = 0.1
    elif (time_elapsed < 25):
      # Move right
      twist.linear.y = -0.1
    elif (time_elapsed < 55):
      # Move left
      twist.linear.y = 0.1
    elif (time_elapsed < 85):
      # Move foward
      twist.linear.x = 0.1

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
