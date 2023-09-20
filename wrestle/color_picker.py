# https://stackoverflow.com/a/58194879

import cv2
import sys
import numpy as np
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

def nothing(x):
    pass

# Create a window
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('SMin','image',0,255,nothing)
cv2.createTrackbar('VMin','image',0,255,nothing)
cv2.createTrackbar('HMax','image',0,179,nothing)
cv2.createTrackbar('SMax','image',0,255,nothing)
cv2.createTrackbar('VMax','image',0,255,nothing)

# Red high
# cv2.setTrackbarPos('HMin', 'image', 140)
# cv2.setTrackbarPos('SMin', 'image', 60)
# cv2.setTrackbarPos('VMin', 'image', 50)
# cv2.setTrackbarPos('HMax', 'image', 180)
# cv2.setTrackbarPos('SMax', 'image', 255)
# cv2.setTrackbarPos('VMax', 'image', 255)

# Red low
cv2.setTrackbarPos('HMin', 'image', 0)
cv2.setTrackbarPos('SMin', 'image', 70)
cv2.setTrackbarPos('VMin', 'image', 135)
cv2.setTrackbarPos('HMax', 'image', 17)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# cv2.setTrackbarPos('HMin', 'image', 0)
# cv2.setTrackbarPos('SMin', 'image', 0)
# cv2.setTrackbarPos('VMin', 'image', 0)
# cv2.setTrackbarPos('HMax', 'image', 90)
# cv2.setTrackbarPos('SMax', 'image', 65)
# cv2.setTrackbarPos('VMax', 'image', 180)

# Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

wait_time = 33

def color_pick(image):
    global phMin
    global phMax
    global psMin
    global psMax
    global pvMin
    global pvMax
    # Load in image
    # image = cv2.imread('images/130.png')

    output = image

    # get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin','image')
    sMin = cv2.getTrackbarPos('SMin','image')
    vMin = cv2.getTrackbarPos('VMin','image')

    hMax = cv2.getTrackbarPos('HMax','image')
    sMax = cv2.getTrackbarPos('SMax','image')
    vMax = cv2.getTrackbarPos('VMax','image')

    # Set minimum and max HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Create HSV Image and threshold into a range.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(image,image, mask= mask)

    # Print if there is a change in HSV value
    if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax

    # Display output image
    cv2.imshow('image',output)
    cv2.waitKey(1)

    # Wait longer to prevent freeze for videos.
    # if cv2.waitKey(wait_time) & 0xFF == ord('q'):
    #     break

    # cv2.destroyAllWindows()

class RobotDetection(Node):
  """
  Create an RobotDetection class, which is a subclass of the Node class.
  """
  def __init__(self):
    super().__init__('robot_detection')
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image,
      'image_bot',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function.
    """
    self.data = data
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image
    img = self.br.imgmsg_to_cv2(data)

    color_pick(img)

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  robot_detection = RobotDetection()
  rclpy.spin(robot_detection)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  robot_detection.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
