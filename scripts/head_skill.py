#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from vision_msgs.msg import BoundingBox2D
from wrestle.image_processing import ImageProcessing
from nao_lola_command_msgs.msg import JointIndexes, JointPositions
from math import sin, pi, radians

MAX_YAW = 0.5
PERIOD = 5.0

class HeadSkill(Node):

  def __init__(self):

    super().__init__('head_skill')

    self.publisher = self.create_publisher(JointPositions, 'effectors/joint_positions', 10)
    self.timer = self.create_timer(0.02, self.timer_callback)
    self.time_start = self.get_clock().now()

  def timer_callback(self):
    time_since_start = (self.get_clock().now() - self.time_start).nanoseconds / 1000000000.0
    headyaw = MAX_YAW * sin(2.0 * pi * time_since_start / PERIOD)

    msg = JointPositions()
    msg.indexes = [JointIndexes.HEADPITCH, JointIndexes.HEADYAW]
    msg.positions = [radians(25), headyaw]
    self.publisher.publish(msg)

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
