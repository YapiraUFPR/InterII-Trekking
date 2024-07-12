#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2 

from std_msgs.msg import Bool

import Jetson.GPIO as GPIO

class FlareListener(Node):

    def __init__(self):
        super().__init__('flare_listener')
        self.get_logger().info('Starting LED listener...')

        # Parse parameters
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        flare_config = fs.getNode("actuators").getNode("flare")
        self.topic = flare_config.getNode("topic").string()
        self.pin = int(flare_config.getNode("pin").real())
        fs.release()

        # Start jetson GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

        self.flare_subscriber = self.create_subscription(Bool, self.topic, self.flare_callback, 10)

        self.get_logger().info("Initialize flare subscriber")

    def flare_callback(self, msg):

        # self.get_logger().info(f"Received msg: {msg.data}")

        value = GPIO.HIGH if msg.data else GPIO.LOW
        GPIO.output(self.pin, value)

if __name__ == "__main__":
    rclpy.init(args=None)

    flare_listener = FlareListener()

    rclpy.spin(flare_listener)

    flare_listener.destroy_node()
    rclpy.shutdown()
