#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped, Twist
from math import atan2, radians, sqrt
from naosoccer_pos_action_interfaces.action import Action as PosAction
from nao_lola_sensor_msgs.msg import Accelerometer, Buttons
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time
from walk_interfaces.action import Walk
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data

ACCELEROMETER_FALLEN = 7.0  # m/s/s


class MotionManager(Node):
  def __init__(self):
    super().__init__('motion_manager')

    # Timer
    self.get_logger().info("Initialize timer")
    self.timer = self.create_timer(0.02, self.timer_callback)

    # Action clients
    self.get_logger().info("Initialize action clients")
    self.walk_client = ActionClient(self, Walk, 'walk')
    self.getup_front_client = ActionClient(self, PosAction, 'getup_front')
    self.getup_back_client = ActionClient(self, PosAction, 'getup_back')
    self.tip_over_client = ActionClient(self, PosAction, 'tip_over')
    self.crouch_client = ActionClient(self, PosAction, 'crouch')
    self.punch_client = ActionClient(self, PosAction, 'punch')

    # Wait for action servers to come up
    self.get_logger().info("Wait for action servers")
    if not self.walk_client.wait_for_server(timeout_sec=5.0):
      self.get_logger().error("Failed to connect to walk action server")
    if not self.getup_front_client.wait_for_server(timeout_sec=5.0):
      self.get_logger().error("Failed to connect to getup front action server")
    if not self.getup_back_client.wait_for_server(timeout_sec=5.0):
      self.get_logger().error("Failed to connect to getup back action server")
    if not self.tip_over_client.wait_for_server(timeout_sec=5.0):
      self.get_logger().error("Failed to connect to tip over action server")
    if not self.crouch_client.wait_for_server(timeout_sec=5.0):
      self.get_logger().error("Failed to connect to crouch action server")
    if not self.punch_client.wait_for_server(timeout_sec=5.0):
      self.get_logger().error("Failed to connect to punch left action server")

    # Subscriptions
    self.get_logger().info("Initialize subscriptions")
    self.opponent_subscription = self.create_subscription(
      PointStamped, 'opponent_point', self.opponent_callback, qos_profile_sensor_data)
    self.accelerometer_subscription = self.create_subscription(
      Accelerometer, 'sensors/accelerometer', self.accelerometer_callback, qos_profile_sensor_data)
    self.buttons_subscription = self.create_subscription(
      Buttons, 'sensors/buttons', self.buttons_callback, qos_profile_sensor_data)

    # Publishers
    self.twist_publisher = self.create_publisher(Twist, 'target', 1)
    self.arm_enable = self.create_publisher(Bool, 'arm_provider/enable', 1)

    # Some states
    self.get_logger().info("Initialize some states")
    self.opponent_heading_average = 0  # radians
    self.opponent_distance_average = 3.0  # metres
    self.fall_direction = None  # None, 'front', 'back', 'left', 'right'
    self.time_crouch_requested = Time(clock_type=ClockType.ROS_TIME)
    self.last_time_opponent_detected = Time(clock_type=ClockType.ROS_TIME)
    self.last_time_bumper_left_pressed = self.get_clock().now()
    self.last_time_bumper_right_pressed = self.get_clock().now()
    self.acc_x_avg = 0  # radians
    self.acc_y_avg = 0  # radians
    self.doing_action = False
    self.crouched = False
    self.should_punch_counter = 0

    self.walk_goal_handle = None
    self.walking = False

    self.should_spin = False

  def timer_callback(self):

    if self.doing_action:
      return

    if not self.crouched:
      self.action_future = self.crouch_client.send_goal_async(PosAction.Goal())
      self.action_future.add_done_callback(self.action_goal_response_callback)
      self.doing_action = True
      self.crouched = True
      self.time_crouch_requested = self.get_clock().now()
      return

    if self.fall_direction is not None:
      if self.fall_direction == 'front':
        self.get_logger().info("Getup Front")
        self.action_future = self.getup_front_client.send_goal_async(PosAction.Goal())
      if self.fall_direction == 'back':
        self.get_logger().info("Getup Back")
        self.action_future = self.getup_back_client.send_goal_async(PosAction.Goal())
      if self.fall_direction in ('left', 'right'):
        self.get_logger().info("Tip Over")
        self.action_future = self.tip_over_client.send_goal_async(PosAction.Goal())
      self.action_future.add_done_callback(self.action_goal_response_callback)
      self.cancel_walk()
      self.arm_enable.publish(Bool(data=False))
      self.doing_action = True
      return

    if self.should_punch():
      self.action_future = self.punch_client.send_goal_async(PosAction.Goal())
      self.action_future.add_done_callback(self.action_goal_response_callback)
      self.cancel_walk()
      self.arm_enable.publish(Bool(data=False))
      self.doing_action = True
      return

    if not self.walking:
      self.get_logger().info("Walk")
      self.walk_future = self.walk_client.send_goal_async(Walk.Goal())
      self.walk_future.add_done_callback(self.walk_goal_response_callback)
      self.walking = True
      self.arm_enable.publish(Bool(data=True))
    self.twist_publisher.publish(self.calculate_twist())

  def opponent_callback(self, opponent_point):
    heading = atan2(opponent_point.point.y, opponent_point.point.x)
    distance = sqrt(opponent_point.point.x ** 2 + opponent_point.point.y ** 2)
    self.opponent_heading_average = self.opponent_heading_average * 0.5 + heading * 0.5
    self.opponent_distance_average = self.opponent_distance_average * 0.5 + distance * 0.5
    # self.get_logger().info(f"Opponent distance average: {self.opponent_distance_average}")
    self.last_time_opponent_detected = Time.from_msg(opponent_point.header.stamp)

  def accelerometer_callback(self, accelerometer):
    self.acc_x_avg = self.acc_x_avg * 0.9 + accelerometer.x * 0.1
    self.acc_y_avg = self.acc_y_avg * 0.9 + accelerometer.y * 0.1

    if self.acc_x_avg > ACCELEROMETER_FALLEN:
      self.fall_direction = 'back'
    elif self.acc_x_avg < -ACCELEROMETER_FALLEN:
      self.fall_direction = 'front'
    elif self.acc_y_avg > ACCELEROMETER_FALLEN:
      self.fall_direction = 'left'
    elif self.acc_y_avg < -ACCELEROMETER_FALLEN:
      self.fall_direction = 'right'
    else:
      self.fall_direction = None

  def buttons_callback(self, buttons):
    if buttons.l_foot_bumper_left or buttons.l_foot_bumper_right:
      self.last_time_bumper_left_pressed = self.get_clock().now()
    if buttons.r_foot_bumper_left or buttons.r_foot_bumper_right:
      self.last_time_bumper_right_pressed = self.get_clock().now()

  def action_goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      return
    self._action_get_result_future = goal_handle.get_result_async()
    self._action_get_result_future.add_done_callback(self.action_result_callback)

  def action_result_callback(self, _):
    self.doing_action = False

  def cancel_walk(self):
    if self.walk_goal_handle is not None:
      self.walk_goal_handle.cancel_goal_async()
      self.walking = False

  def should_punch(self):
    time_elapsed_since_opponent_detected = \
      (self.get_clock().now() - self.last_time_opponent_detected).nanoseconds / 1e9

    if time_elapsed_since_opponent_detected < 0.5 and self.opponent_distance_average < 0.3 and \
      abs(self.opponent_heading_average) < radians(20):
      self.should_punch_counter += 1
    else:
      self.should_punch_counter -= 1
    self.should_punch_counter = min(max(self.should_punch_counter, 0), 5)

    if self.should_punch_counter == 5:
      return True
    return False

  def calculate_twist(self):
    time_elapsed_since_opponent_detected = \
      (self.get_clock().now() - self.last_time_opponent_detected).nanoseconds / 1e9
    time_elapsed_since_crouch_finished = \
      (self.get_clock().now() - self.time_crouch_requested).nanoseconds / 1e9
    time_elapsed_since_bumper_left_pressed = \
      (self.get_clock().now() - self.last_time_bumper_left_pressed).nanoseconds / 1e9
    time_elapsed_since_bumper_right_pressed = \
      (self.get_clock().now() - self.last_time_bumper_right_pressed).nanoseconds / 1e9

    # Update self.should_spin hysteresis
    if self.should_spin and abs(self.opponent_heading_average) < radians(5):
      self.should_spin = False
    elif not self.should_spin and abs(self.opponent_heading_average) > radians(10):
      self.should_spin = True

    twist = Twist()
    if time_elapsed_since_crouch_finished < 5.0:
      # Walk sideways
      # self.get_logger().info("Walk sideways")
      twist.linear.y = 0.3
    elif time_elapsed_since_crouch_finished < 9.0:
      # Walk forwards
      # self.get_logger().info("Walk forwards")
      twist.linear.x = 0.2
    elif time_elapsed_since_opponent_detected > 2.0:
      # Slowly turn in direction we think opponent is in
      # self.get_logger().info("Slowly turn in direction we think opponent is in")
      twist.angular.z = 1.0 if self.opponent_heading_average > 0 else -1.0
    elif self.should_spin:
      # Quickly turn towards opponent
      # self.get_logger().info("Quickly turn towards opponent")
      twist.angular.z = 1.0 if self.opponent_heading_average > 0 else -1.0
    elif time_elapsed_since_bumper_left_pressed < 1.0 or time_elapsed_since_bumper_right_pressed < 1.0:
      # Don't walk into obstacle detected by foot bumper
      # self.get_logger().info("Don't walk into obstacle detected by foot bumper")
      pass
    elif self.opponent_distance_average > 0.3:
      # Ram into opponent
      # self.get_logger().info("Ram into opponent")
      twist.linear.x = 0.2
      twist.angular.z = self.opponent_heading_average
    # else:
      # self.get_logger().info("Close to opponent, not walking in")

    return twist

  def walk_goal_response_callback(self, future):
    self.walk_goal_handle = future.result()

def main(args=None):
  rclpy.init(args=args)
  motion_manager = MotionManager()
  rclpy.spin(motion_manager)
  motion_manager.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
