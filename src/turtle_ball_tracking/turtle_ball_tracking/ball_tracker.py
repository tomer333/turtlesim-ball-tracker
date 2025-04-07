import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.pose_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.bridge = CvBridge()
        self.img_width = 500
        self.img_height = 500
        self.turtle_world_size = 11
        self.turtle_world_lower_limit = 0.5
        self.turtle_world_upper_limit = 10.5

        self.current_x = 5.5
        self.current_y = 5.5
        self.current_theta = 0.0

        self.turtle_linear_speed = 3.5
        self.turtle_angular_speed = 4.0

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            ball_contour = max(contours, key=cv2.contourArea)
            (ball_x, ball_y), radius = cv2.minEnclosingCircle(ball_contour)

            ball_x_mapped = (ball_x / self.img_width) * self.turtle_world_size
            ball_y_mapped = (1 - (ball_y / self.img_height)) * \
                self.turtle_world_size

            self.track_and_move_turtle(ball_x_mapped, ball_y_mapped)

    def track_and_move_turtle(self, ball_x, ball_y):
        target_angle = math.atan2(
            ball_y - self.current_y, ball_x - self.current_x)

        angle_error = target_angle - self.current_theta

        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        vel_msg = Twist()

        vel_msg.angular.z = self.turtle_angular_speed * angle_error

        next_x = self.current_x + \
            self.turtle_world_lower_limit * math.cos(target_angle)
        next_y = self.current_y + \
            self.turtle_world_lower_limit * math.sin(target_angle)

        if self.turtle_world_lower_limit <= next_x <= self.turtle_world_upper_limit and \
                self.turtle_world_lower_limit <= next_y <= self.turtle_world_upper_limit:
            vel_msg.linear.x = self.turtle_linear_speed
        else:
            vel_msg.linear.x = 0.0

        self.publisher_.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
