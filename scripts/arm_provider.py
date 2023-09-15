#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from biped_interfaces.msg import SolePoses
from nao_lola_command_msgs.msg import JointIndexes, JointPositions
from math import radians, degrees

class ArmProvider(Node):

  def __init__(self):

    super().__init__('arm_provider')
    self.subscription = self.create_subscription(SolePoses, 'motion/sole_poses', self.sp_callback, 10)
    self.publisher = self.create_publisher(JointPositions, 'effectors/joint_positions', 10)

  def sp_callback(self, sole_poses):
    msg = JointPositions()
    msg.indexes = [JointIndexes.LSHOULDERPITCH, JointIndexes.RSHOULDERPITCH,
                   JointIndexes.LSHOULDERROLL, JointIndexes.RSHOULDERROLL]

    mult = 10.0
    l_shoulder_pitch = radians(90) + mult * sole_poses.l_sole.position.x
    r_shoulder_pitch = radians(90) + mult * sole_poses.r_sole.position.x
    print("l_shoulder_pitch(deg): ", degrees(l_shoulder_pitch))
    msg.positions = [l_shoulder_pitch, r_shoulder_pitch, radians(7), radians(-7)]
    self.publisher.publish(msg)

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  arm_provideer = ArmProvider()

  # Spin the node so the callback function is called.
  rclpy.spin(arm_provideer)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  arm_provideer.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
