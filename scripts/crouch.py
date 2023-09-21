#!/usr/bin/env python3

import rclpy
from walk_interfaces.action import Crouch as CrouchAction
from rclpy.action import ActionServer
from rclpy.node import Node
from biped_interfaces.msg import SolePoses

CROUCH_PERIOD = 0.5

START_X = 0.0
START_Z = -0.333

END_X = -0.03
END_Z = -0.29

class Crouch(Node):

  def __init__(self):

    super().__init__('crouch')

    self.pub_sole_poses = self.create_publisher(SolePoses, 'motion/sole_poses', 10)

    self._action_server = ActionServer(
      self, CrouchAction, 'crouch', self.execute_callback)

  def execute_callback(self, goal_handle):
    # self.get_logger().info('Executing crouch...')
    result = CrouchAction.Result()

    self.time_start = self.get_clock().now()

    time_since_start = 0.0

    while time_since_start < CROUCH_PERIOD:
      progress = time_since_start / CROUCH_PERIOD
      # self.get_logger().info(f'Progress: {progress}')

      x = START_X * (1 - progress) + END_X * progress
      z = START_Z * (1 - progress) + END_Z * progress

      sole_poses = SolePoses()
      sole_poses.l_sole.position.x = x
      sole_poses.l_sole.position.y = 0.05
      sole_poses.l_sole.position.z = z
      sole_poses.r_sole.position.x = x
      sole_poses.r_sole.position.y = -0.05
      sole_poses.r_sole.position.z = z
      self.pub_sole_poses.publish(sole_poses)

      time_since_start = (self.get_clock().now() - self.time_start).nanoseconds / 1000000000.0

    # self.get_logger().info('Done crouch...')
    goal_handle.succeed()
    return result


def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  crouch = Crouch()

  # Spin the node so the callback function is called.
  rclpy.spin(crouch)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  crouch.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()

