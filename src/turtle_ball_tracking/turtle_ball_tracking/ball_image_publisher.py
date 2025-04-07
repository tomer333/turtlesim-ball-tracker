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

        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.width = 500  
        self.height = 500  

        self.ball_x = self.width // 2  
        self.ball_y = self.height // 2  

        self.ball_radius = 15  

        angle = random.uniform(0, 2 * np.pi)  
        self.ball_speed = 5 
        self.ball_speed_x = self.ball_speed * np.cos(angle)
        self.ball_speed_y = self.ball_speed * np.sin(angle)

        self.timer = self.create_timer(1.0 / 30, self.publish_frame)

    def publish_frame(self):
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        cv2.circle(frame, (int(self.ball_x), int(self.ball_y)), self.ball_radius, (0, 0, 255), -1)

        self.ball_x += self.ball_speed_x
        self.ball_y += self.ball_speed_y

        if self.ball_x >= self.width - self.ball_radius or self.ball_x <= self.ball_radius:
            self.ball_speed_x = -self.ball_speed_x
        
        if self.ball_y >= self.height - self.ball_radius or self.ball_y <= self.ball_radius:
            self.ball_speed_y = -self.ball_speed_y

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = BallImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
