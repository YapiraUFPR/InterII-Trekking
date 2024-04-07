#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from drivers.libs.gy_tcs3200 import TCS3200

import cv2
from time import sleep

class Tcs32Publisher(Node):

    def __init__(self):
        super().__init__('tcs32_publisher')
        self.logger = self.get_logger()
        self.logger.info('Initializing color sensor node...')

        # load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        color_config = fs.getNode("sensors").getNode("color")
        topic = color_config.getNode("topic").string()
        sample_rate = int(color_config.getNode("sample_rate").real())
        pin_s2 = int(color_config.getNode("pins").getNode("s2").real())
        pin_s3 = int(color_config.getNode("pins").getNode("s3").real())
        pin_signal = int(color_config.getNode("pins").getNode("signal").real())

        # sensor initialization
        self.tcs = None
        timeout = 5
        while self.tcs is None:
            self.logger.info('Initializing sensor TCS3200...')
            try:
                self.tcs = TCS3200(pin_s2, pin_s3, pin_signal)
            except Exception as e:
                self.tcs = None
                self.logger.error(f"Failed to initialize TCS3200: {e}")
                self.logger.error(f"Retrying in {timeout} seconds...")
                sleep(timeout)
                timeout *= 2

        # init publishers
        self.publisher = self.create_publisher(ColorRGBA, topic, 10)
        self.timer = self.create_timer(1/sample_rate, self.timer_callback)

        self.logger.info('Distance node launched.')

    def timer_callback(self):
        self.logger.info("Publishing color sensor data...", once=True)

        rgb = self.tcs.read()
        
        msg = ColorRGBA()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.r = (rgb[0])
        msg.g = (rgb[1])
        msg.b = (rgb[2])
        msg.a = 1.0

        self.publisher.publish(msg)

if __name__ == "__main__":
    rclpy.init(args=None)

    tcs_publisher = Tcs32Publisher()
    rclpy.spin(tcs_publisher)
    tcs_publisher.destroy_node()
    rclpy.shutdown()