#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

from drivers.libs.adafruit_vl53l0x import VL53L0X
from drivers.libs.i2c import I2C
import cv2
from time import sleep

class Vl5Publisher(Node):

    def __init__(self):
        super().__init__('vl5_publisher')
        self.logger = self.get_logger()
        self.logger.info('Initializing infrared distance sensor node...')

        # load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        dist_config = fs.getNode("sensors").getNode("distance")
        topic = dist_config.getNode("topic").string()
        sample_rate = int(dist_config.getNode("sample_rate").real())
        i2c_bus = int(dist_config.getNode("bus").real())
        fs.release()

        # sensor initialization
        self.vl5 = None
        timeout = 5
        while self.vl5 is None:
            self.logger.info('Initializing sensor VL53L0X...')
            try:
                i2c = I2C(i2c_bus, 400000)
                self.vl5 = VL53L0X(i2c, address=0x29)
            except Exception as e:
                self.vl5 = None
                self.logger.error(f"Failed to initialize VL53L0X: {e}")
                self.logger.error(f"Retrying in {timeout} seconds...")
                sleep(timeout)
                timeout *= 2

        # init publishers
        self.publisher = self.create_publisher(Range, topic, 10)
        self.timer = self.create_timer(1/sample_rate, self.timer_callback)

        self.logger.info('Distance node launched.')

    def timer_callback(self):
        self.logger.info("Publishing IR sensor data...", once=True)

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.range = self.vl5.range

        self.publisher.publish(msg)

if __name__ == "__main__":
    rclpy.init(args=None)

    vl5_publisher = Vl5Publisher()
    rclpy.spin(vl5_publisher)
    vl5_publisher.destroy_node()
    rclpy.shutdown()