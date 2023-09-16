#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from biped_interfaces.msg import SolePoses
from nao_lola_command_msgs.msg import JointIndexes, JointPositions
from math import radians, degrees
from nao_lola_sensor_msgs.msg import Sonar

class ArmProvider(Node):

  def __init__(self):

    super().__init__('arm_provider')
    self.subscription = self.create_subscription(SolePoses, 'motion/sole_poses', self.sp_callback, 10)
    self.publisher = self.create_publisher(JointPositions, 'effectors/joint_positions', 10)
    self.sonar_subscription = self.create_subscription(Sonar, 'sensors/sonar', self.sonar_callback, 10)
    self.timer = self.create_timer(0.02, self.timer_callback)
    self.sole_poses = None
    self.time_start = self.get_clock().now()
    self.obstacle_in_front = False

  def sp_callback(self, sole_poses):
    self.sole_poses = sole_poses

  def sonar_callback(self, sonar):
    time_since_start = (self.get_clock().now() - self.time_start).nanoseconds / 1000000000.0
    if time_since_start < 4.0:
      return
    self.obstacle_in_front = sonar.left < 0.5 or sonar.right < 0.5

  def timer_callback(self):
    if self.sole_poses is None:
      return

    msg = JointPositions()
    msg.indexes = [JointIndexes.LSHOULDERPITCH, JointIndexes.RSHOULDERPITCH,
                   JointIndexes.LSHOULDERROLL, JointIndexes.RSHOULDERROLL]

    if self.obstacle_in_front:
      msg.positions = [radians(0), radians(0), radians(7), radians(-7)]
    else:
      # Calculate from sole pose
      mult = 10.0
      l_shoulder_pitch = radians(90) + mult * self.sole_poses.l_sole.position.x
      r_shoulder_pitch = radians(90) + mult * self.sole_poses.r_sole.position.x
      msg.positions = [l_shoulder_pitch, r_shoulder_pitch, radians(7), radians(-7)]
    self.publisher.publish(msg)

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
