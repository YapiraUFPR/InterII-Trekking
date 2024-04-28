#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from drivers.libs.adafruit_tcs34725 import TCS34725
from drivers.libs.i2c import I2C

import cv2
from time import sleep

class Tcs34Publisher(Node):

    def __init__(self):
        super().__init__('tcs34_publisher')
        self.logger = self.get_logger()
        self.logger.info('Initializing color sensor node....')

        # load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        color_config = fs.getNode("sensors").getNode("color")
        topic = color_config.getNode("topic").string()
        sample_rate = int(color_config.getNode("sample_rate").real())
        i2c_bus = int(color_config.getNode("i2c_bus").real())
        gain = int(color_config.getNode("gain").real())
        integration_time = int(color_config.getNode("integration_time").real())

        # sensor initialization
        self.tcs = None
        timeout = 5
        while self.tcs is None:
            self.logger.info('Initializing sensor TCS34725...')
            try:
                i2c = I2C(i2c_bus, sample_rate*1000)
                self.tcs = TCS34725(i2c, address=0x29)
                self.tcs.gain = gain
                self.tcs.integration_time = integration_time
            except Exception as e:
                self.tcs = None
                self.logger.error(f"Failed to initialize TCS34725: {e}")
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
        
        color_bytes = tcs.color_rgb_bytes
        msg = ColorRGBA()
        msg.r = float(color_bytes[0])
        msg.g = float(color_bytes[1])
        msg.b = float(color_bytes[2])
        msg.a = 1.0

        self.publisher.publish(msg)

if __name__ == "__main__":
    rclpy.init(args=None)

    tcs_publisher = Tcs34Publisher()
    rclpy.spin(tcs_publisher)
    tcs_publisher.destroy_node()
    rclpy.shutdown()