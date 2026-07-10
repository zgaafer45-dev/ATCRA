#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv


class ESP32Camera(Node):

    def __init__(self):
        super().__init__('esp32_camera')

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.cap = cv.VideoCapture("http://192.168.4.1:81/stream")

        self.timer = self.create_timer(0.05, self.publish_frame)  # 20 FPS

    def publish_frame(self):

        # flush old frames (important for ESP32)
        for _ in range(2):
            self.cap.grab()

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("No frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ESP32Camera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()