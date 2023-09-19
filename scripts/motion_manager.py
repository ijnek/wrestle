#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from walk_interfaces.action import Walk
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, PointStamped
from math import atan2
from walk_interfaces.action import Crouch
from math import radians, asin
from naosoccer_pos_action_interfaces.action import Action as PosAction
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool

class MotionManager(Node):

  def __init__(self):
    super().__init__('motion_manager')

    action_cb_group = MutuallyExclusiveCallbackGroup()
    timer_cb_group = MutuallyExclusiveCallbackGroup()

    self.timer = self.create_timer(0.02, self.timer_callback, callback_group=timer_cb_group)
    self.walk_client = ActionClient(self, Walk, 'walk', callback_group=action_cb_group)
    self.getup_front_client = ActionClient(self, PosAction, 'getup_front', callback_group=action_cb_group)
    self.getup_back_client = ActionClient(self, PosAction, 'getup_back', callback_group=action_cb_group)

    self.opponent_subscription = self.create_subscription(PointStamped, 'opponent_point',
                                                          self.opponent_callback, 10)
    self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

    self.arm_enable = self.create_publisher(Bool, 'arm_provider/enable', 10)

    self.crouch_action_client = ActionClient(self, Crouch, 'motion/crouch')
    self.crouch_action_client.wait_for_server(timeout_sec=5.0)

    self.time_crouch_completed = None

    self.last_twist = None

    self.last_time_opponent_detected = self.get_clock().now()
    self.opponent_heading_average = 0.0  # rad
    self.opponent_distance = 10000.0  # m

    self.initial = True
    self.crouched = False
    self.spin = False
    self.doing_getup = False
    self.pitch = 0.0

    self._walk_goal_handle = None

  def opponent_callback(self, opponent_point):
    # Decide on twist here

    heading = atan2(opponent_point.point.y, opponent_point.point.x)
    # self.get_logger().info('Heading: %f' % heading)
    self.opponent_heading_average = self.opponent_heading_average * 0.7 + heading * 0.3
    # print("self.opponent_heading_average: ", self.opponent_heading_average)
    self.last_time_opponent_detected = Time.from_msg(opponent_point.header.stamp)

  def crouch_goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      return
    self._crouch_get_result_future = goal_handle.get_result_async()
    self._crouch_get_result_future.add_done_callback(self.crouch_result_callback)

  def crouch_result_callback(self, _):
    self.crouched = True
    self.time_crouch_completed = self.get_clock().now()

  def getup_goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      return
    self._getup_get_result_future = goal_handle.get_result_async()
    self._getup_get_result_future.add_done_callback(self.getup_result_callback)

  def getup_result_callback(self, _):
    self.doing_getup = False

  def walk_goal_response_callback(self, future):
    self._walk_goal_handle = future.result()
    if not self._walk_goal_handle.accepted:
      return

  def cancel_walk(self):
    # self.get_logger().info('Canceling goal')
    if self._walk_goal_handle is not None:
      self._walk_goal_handle.cancel_goal_async()

  def timer_callback(self):
    if self.initial:
      self.arm_enable.publish(Bool(data=True))
      self._send_goal_future = self.crouch_action_client.send_goal_async(Crouch.Goal())
      self._send_goal_future.add_done_callback(self.crouch_goal_response_callback)
      self.initial = False
      return

    if not self.crouched:
      # In the process of crouching, don't do anything here
      return

    if self.doing_getup:
      # In the process of getting up, don't do anything here
      return

    if self.pitch > 1.0:
      # self.get_logger().info("do getup front")
      self.doing_getup = True
      self.cancel_walk()
      self.arm_enable.publish(Bool(data=False))
      self.getup_future = self.getup_front_client.send_goal_async(PosAction.Goal())
      self.getup_future.add_done_callback(self.getup_goal_response_callback)
      return

    if self.pitch < -1.0:
      # self.get_logger().info("do getup back")
      self.doing_getup = True
      self.cancel_walk()
      self.arm_enable.publish(Bool(data=False))
      self.getup_future = self.getup_back_client.send_goal_async(PosAction.Goal())
      self.getup_future.add_done_callback(self.getup_goal_response_callback)
      return

    self.arm_enable.publish(Bool(data=True))
    twist = Twist()
    time_elapsed_since_opponent_detected = \
      (self.get_clock().now() - self.last_time_opponent_detected).nanoseconds / 1e9
    time_elapsed_since_crouch_finished = \
      (self.get_clock().now() - self.time_crouch_completed).nanoseconds / 1e9

    if time_elapsed_since_opponent_detected > 2.0:
      self.spin = True
    elif not self.spin and abs(self.opponent_heading_average) > radians(20):
      # print("spin!")
      self.spin = True
    elif self.spin and abs(self.opponent_heading_average) < radians(10):
      # print("not spin!")
      self.spin = False

    if time_elapsed_since_crouch_finished > 0.2:
      if self.spin:
        twist.angular.z = 1.0 if self.opponent_heading_average > 0 else -1.0
      else:
        twist.linear.x = 0.4
        twist.angular.z = 0.0

    walk_goal = Walk.Goal()
    walk_goal.twist = twist
    self.walk_future = self.walk_client.send_goal_async(walk_goal)
    self.walk_future.add_done_callback(self.walk_goal_response_callback)
    self.last_twist = twist

  def imu_callback(self, imu):
    # Calculate pitch here
    qx = imu.orientation.x
    qy = imu.orientation.y
    qz = imu.orientation.z
    qw = imu.orientation.w
    [roll, pitch, yaw] = euler_from_quaternion([qx, qy, qz, qw])
    self.pitch = pitch
    # self.get_logger().info(f"pitch: {pitch}")

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
