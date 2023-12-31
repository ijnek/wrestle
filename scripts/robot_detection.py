#!/usr/bin/env python3

# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from wrestle.image_processing2 import locate_opponent
from visualization_msgs.msg import Marker
from ipm_interfaces.srv import MapPoint
from ipm_library.utils import create_horizontal_plane
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data

ROBOT_HEIGHT = 0.58

class RobotDetection(Node):
  """
  Create an RobotDetection class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('robot_detection')

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 'image', self.listener_callback, qos_profile_sensor_data)
    self.subscription # prevent unused variable warning

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self.marker_publisher = self.create_publisher(Marker, 'opponent', 1)

    self.ipm_client = self.create_client(MapPoint, 'map_point')
    while not self.ipm_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    self.point_publisher = self.create_publisher(PointStamped, 'opponent_point', 1)

    self.get_logger().info('Robot detection constructor')

  def listener_callback(self, data):
    """
    Callback function.
    """
    self.data = data
    # Display the message on the console
    self.get_logger().info('Correctly receiving video frame', throttle_duration_sec=10)

    # Convert ROS Image message to OpenCV image
    img = self.br.imgmsg_to_cv2(data)

    opponent_bb = locate_opponent(img)

    if opponent_bb is None:
      return

    self.get_logger().info('Detecting opponent correctly, calling ipm', throttle_duration_sec=10)
    req = MapPoint.Request()
    req.plane = create_horizontal_plane()
    req.point.x = opponent_bb.center.position.x
    req.point.y = opponent_bb.center.position.y + 0.5 * opponent_bb.size_y
    req.time = data.header.stamp
    req.plane_frame_id = 'base_footprint'
    req.output_frame_id = 'base_footprint'
    # print(req)
    future = self.ipm_client.call_async(req)
    future.add_done_callback(self.ipm_done_callback)

  def ipm_done_callback(self, future):
    self.get_logger().info('Correctly calling ipm_done_callback', throttle_duration_sec=10)
    resp = future.result()
    max_detection_dist = 4.0
    if resp.point.point.x**2 + resp.point.point.y**2 > max_detection_dist**2:
      # Ignore the detection, too far away and probably false positive
      # self.get_logger().info('false positive!')
      return

    if resp.result == MapPoint.Response.RESULT_SUCCESS:
      self.point_publisher.publish(resp.point)
      self.publish_robot_marker(resp.point)
      self.get_logger().info('Correctly publishing robot point', throttle_duration_sec=10)
    else:
      self.get_logger().error(f"mapping failed. status: {resp.result}", throttle_duration_sec=10)

  def publish_robot_marker(self, point):
    marker = Marker()
    marker.header = point.header
    marker.type = Marker.CUBE
    marker.pose.position = point.point
    marker.pose.position.z = marker.pose.position.z + ROBOT_HEIGHT / 2.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = ROBOT_HEIGHT
    marker.color.r = 1.0
    marker.color.a = 1.0
    self.marker_publisher.publish(marker)

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  robot_detection = RobotDetection()

  # Spin the node so the callback function is called.
  rclpy.spin(robot_detection)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  robot_detection.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
