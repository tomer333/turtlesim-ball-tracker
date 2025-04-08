import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from turtle_ball_tracking.ball_detector import BallDetector
from turtle_ball_tracking.pid_controller import PIDController


class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')

        self._ball_image_subscriber = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10)

        self._turtle_pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self._pose_callback, 10)

        self._turtle_pos_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        self._bridge = CvBridge()
        self._ball_detector = BallDetector(500, 500)
        self._pid_controller_angle = PIDController(3.0, 0.0, 0.5)
        self._pid_controller_distance = PIDController(2.0, 0.0, 0.5)

        self._turtle_world_lower_limit = 0.5
        self._turtle_world_upper_limit = 10.5

        self._current_x = 5.5
        self._current_y = 5.5
        self._current_theta = 0.0

    def _pose_callback(self, msg):
        self._current_x = msg.x
        self._current_y = msg.y
        self._current_theta = msg.theta

    def _image_callback(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        ball_x, ball_y = self._ball_detector.detect_ball(frame)
        if ball_x is not None and ball_y is not None:
            self._track_and_move_turtle(ball_x, ball_y)

    def _track_and_move_turtle(self, ball_x, ball_y):
        target_angle = math.atan2(
            ball_y - self._current_y, ball_x - self._current_x)

        angle_error = target_angle - self._current_theta

        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        vel_msg = Twist()

        angular_velocity = self._pid_controller_angle.compute(angle_error)
        distance_error = math.sqrt(
            (ball_x - self._current_x) ** 2 + (ball_y - self._current_y) ** 2)
        linear_velocity = self._pid_controller_distance.compute(distance_error)

        vel_msg.angular.z = angular_velocity

        next_x = self._current_x + \
            self._turtle_world_lower_limit * math.cos(target_angle)
        next_y = self._current_y + \
            self._turtle_world_lower_limit * math.sin(target_angle)

        if self._turtle_world_lower_limit <= next_x <= self._turtle_world_upper_limit and \
                self._turtle_world_lower_limit <= next_y <= self._turtle_world_upper_limit:
            vel_msg.linear.x = linear_velocity
        else:
            vel_msg.linear.x = 0.0

        self._turtle_pos_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
