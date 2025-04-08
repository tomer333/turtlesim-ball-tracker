import cv2
import numpy as np


class BallDetector:
    def __init__(self, img_width, img_height):
        self._img_width = img_width
        self._img_height = img_height

    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            ball_contour = max(contours, key=cv2.contourArea)
            (ball_x, ball_y), radius = cv2.minEnclosingCircle(ball_contour)
            ball_x_mapped = (ball_x / self._img_width) * 11
            ball_y_mapped = (1 - (ball_y / self._img_height)) * 11
            return ball_x_mapped, ball_y_mapped
        return None, None
