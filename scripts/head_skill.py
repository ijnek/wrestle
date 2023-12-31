#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from nao_lola_command_msgs.msg import JointIndexes, JointPositions
from math import sin, pi, radians, atan2, sqrt
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data

MAX_YAW = 0.0
PERIOD = 3.0

class HeadSkill(Node):

  def __init__(self):

    super().__init__('head_skill')

    self.publisher = self.create_publisher(JointPositions, 'effectors/joint_positions', 10)
    self.timer = self.create_timer(0.02, self.timer_callback)
    self.time_start = self.get_clock().now()
    self.opponent_subscription = self.create_subscription(
      PointStamped, 'opponent_point', self.opponent_callback, qos_profile_sensor_data)
    self.time_since_obstacle_detected = None
    self.opponent_heading = 0
    self.opponent_distance = 5.0

  def timer_callback(self):
    if self.time_since_obstacle_detected is not None:
      look_at_opponent = ((self.get_clock().now() - self.time_since_obstacle_detected).nanoseconds / 1e9) < 1.0
    else:
      look_at_opponent = False
    # self.get_logger().info(f'look_at_opponent: {look_at_opponent}')

    msg = JointPositions()

    if look_at_opponent:
      msg.indexes = [JointIndexes.HEADPITCH, JointIndexes.HEADYAW]
      head_pitch = radians(-10)
      if (self.opponent_distance < 1.5):
        head_pitch += radians(15) * (1.5 - self.opponent_distance)
      msg.positions = [head_pitch, min(max(self.opponent_heading, -0.5), 0.5)]
    else:
      time_since_start = (self.get_clock().now() - self.time_start).nanoseconds / 1000000000.0
      headyaw = MAX_YAW * sin(2.0 * pi * time_since_start / PERIOD)
      msg.indexes = [JointIndexes.HEADPITCH, JointIndexes.HEADYAW]
      msg.positions = [radians(-10), headyaw]
    self.publisher.publish(msg)

  def opponent_callback(self, opponent_point):
    # time_since_start = (self.get_clock().now() - self.time_start).nanoseconds / 1000000000.0
    # if time_since_start < 4.0:
    #   return

    self.time_since_obstacle_detected = self.get_clock().now()
    self.opponent_heading = atan2(opponent_point.point.y, opponent_point.point.x)
    self.opponent_distance = sqrt(opponent_point.point.x ** 2 + opponent_point.point.y ** 2)

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  head_skill = HeadSkill()

  # Spin the node so the callback function is called.
  rclpy.spin(head_skill)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  head_skill.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
