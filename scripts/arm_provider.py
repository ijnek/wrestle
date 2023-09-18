#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from biped_interfaces.msg import SolePoses
from nao_lola_command_msgs.msg import JointIndexes, JointPositions
from math import radians, degrees, atan2
from nao_lola_sensor_msgs.msg import Sonar
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

class ArmProvider(Node):

  def __init__(self):

    super().__init__('arm_provider')
    self.subscription = self.create_subscription(SolePoses, 'motion/sole_poses', self.sp_callback, 10)
    self.publisher = self.create_publisher(JointPositions, 'effectors/joint_positions', 10)
    self.opponent_subscription = self.create_subscription(PointStamped, 'opponent_point',
                                                          self.opponent_callback, 10)
    self.enable = self.create_subscription(Bool, 'arm_provider/enable', self.enable_callback, 10)
    self.sole_poses = None
    self.time_start = self.get_clock().now()
    self.obstacle_in_front = False
    self.time_since_obstacle_detected = None
    self.enabled = False
    self.opponent_heading = radians(0)

  def enable_callback(self, enable):
    self.enabled = enable.data

  def sp_callback(self, sole_poses):
    if not self.enabled:
      return False

    msg = JointPositions()
    msg.indexes = [JointIndexes.LSHOULDERPITCH, JointIndexes.RSHOULDERPITCH,
                   JointIndexes.LSHOULDERROLL, JointIndexes.RSHOULDERROLL]

    if self.time_since_obstacle_detected is not None:
      push_obstacle = ((self.get_clock().now() - self.time_since_obstacle_detected).nanoseconds / 1e9) < 1.0
    else:
      push_obstacle = False

    if push_obstacle:
      # self.get_logger().info("Push obstacle!")
      # Put arms up
      msg.positions = [radians(0), radians(0), self.opponent_heading, self.opponent_heading]
    else:
      # Swing arms
      mult = 10.0
      l_shoulder_pitch = radians(90) + mult * sole_poses.l_sole.position.x
      r_shoulder_pitch = radians(90) + mult * sole_poses.r_sole.position.x
      msg.positions = [l_shoulder_pitch, r_shoulder_pitch, radians(7), radians(-7)]
    self.publisher.publish(msg)

  def opponent_callback(self, opponent_point):
    time_since_start = (self.get_clock().now() - self.time_start).nanoseconds / 1000000000.0
    if time_since_start < 4.0:
      return

    if opponent_point.point.x < 0.6:
      self.time_since_obstacle_detected = self.get_clock().now()
      self.opponent_heading = atan2(opponent_point.point.y, opponent_point.point.x)

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  arm_provider = ArmProvider()

  # Spin the node so the callback function is called.
  rclpy.spin(arm_provider)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  arm_provider.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
