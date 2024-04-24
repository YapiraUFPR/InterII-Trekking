#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from adafruit_servokit import ServoKit
from drivers.libs.i2c import I2C
import cv2
from time import sleep

RAD_TO_DEG = 180 / 3.14159265358979323846

class MotorsListener(Node):

    def __init__(self):
        super().__init__('motor_listener')
        self.logger = self.get_logger()
        self.logger.info('Initializing motor listener node...')

        # Load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        motors_config = fs.getNode("motors")
        topic = motors_config.getNode("topic").string()
        brake_topic = motors_config.getNode("brake_topic").string()
        self.use_brake = bool(motors_config.getNode("use_brake").real())
        self.servo_channel = int(motors_config.getNode("servo_channel").real())
        self.esc_channel = int(motors_config.getNode("esc_channel").real())
        self.max_speed = int(motors_config.getNode("max_speed").real()) / 100
        self.brake = False
        fs.release()

        # Init servo kit 
        self.kit = ServoKit(channels=16)
        self.kit.servo[self.servo_channel].angle = 90
        self.kit.continuous_servo[self.esc_channel].throttle = 0

        self.current_angle = 90
        self.current_speed = 0

        # Init subscribers
        self.motors_subscriber = self.create_subscription(Twist, topic, self.motors_callback, 10)
        if self.use_brake:
            self.brake_subscriber = self.create_subscription(Bool, brake_topic, self.brake_callback, 10)

        self.logger.info('Motor listener node launched.')

    def motors_callback(self, msg: Twist):
        self.logger.info("Received motor data...", once=True)

        if self.brake:
            return

        # Convert from rad to degrees
        angle = msg.angular.z * RAD_TO_DEG
        self.current_angle += angle
        self.current_angle = max(0, min(180, self.current_angle))
        self.kit.servo[self.servo_channel].angle = self.current_angle

        self.current_speed += msg.linear.x
        self.current_speed = max(-1, min(1, self.current_speed))
        self.kit.continuous_servo[self.esc_channel].throttle = self.current_speed

    def brake_callback(self, msg: Bool):
        self.logger.info("Received brake signal...")

        if msg.data:
            self.logger.info("Braking...")
            self.kit.continuous_servo[self.esc_channel].throttle = 0
            self.kit.servo[self.servo_channel].angle = 90
            self.current_speed = 0
            self.current_angle = 90

    def __del__(self):
        self.kit.continuous_servo[self.esc_channel].throttle = 0
        self.kit.servo[self.servo_channel].angle = 90

if __name__ == "__main__":
    rclpy.init(args=None)

    motors_listener = MotorsListener()
    rclpy.spin(motors_listener)
    motors_listener.destroy_node()
    rclpy.shutdown()