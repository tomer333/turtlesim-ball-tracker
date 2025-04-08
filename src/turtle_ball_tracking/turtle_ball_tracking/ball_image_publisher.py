import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import random


class BallImagePublisher(Node):
    def __init__(self):
        super().__init__('ball_image_publisher')

        self._ball_image_publisher = self.create_publisher(
            Image, '/camera/image_raw', 10)
        self._bridge = CvBridge()

        self._width = 500
        self._height = 500

        self._ball_x = self._width // 2
        self._ball_y = self._height // 2

        self._ball_radius = 15

        angle = random.uniform(0, 2 * np.pi)
        self._ball_speed = 5
        self._ball_speed_x = self._ball_speed * np.cos(angle)
        self._ball_speed_y = self._ball_speed * np.sin(angle)

        self._timer = self.create_timer(1.0 / 30, self._publish_frame)

    def _publish_frame(self):
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)

        cv2.circle(frame, (int(self._ball_x), int(self._ball_y)),
                   self._ball_radius, (0, 0, 255), -1)

        self._ball_x += self._ball_speed_x
        self._ball_y += self._ball_speed_y

        if self._ball_x >= self._width - self._ball_radius or self._ball_x <= self._ball_radius:
            self._ball_speed_x = -self._ball_speed_x

        if self._ball_y >= self._height - self._ball_radius or self._ball_y <= self._ball_radius:
            self._ball_speed_y = -self._ball_speed_y

        ros_image = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self._ball_image_publisher.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    node = BallImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
