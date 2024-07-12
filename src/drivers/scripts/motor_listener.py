#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from drivers.libs.adafruit_servokit import ServoKit
import cv2
from time import sleep

RAD_TO_DEG = 180 / 3.14159265358979323846
ZERO_ANGLE_CORRECTION = 45

class MotorsListener(Node):

    def __init__(self):
        super().__init__('motor_listener')
        self.logger = self.get_logger()
        self.logger.info('Initializing motor listener node...')

        # Load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        motors_config = fs.getNode("actuators").getNode("motors")
        topic = motors_config.getNode("topic").string()
        brake_topic = motors_config.getNode("brake_topic").string()
        self.use_brake = bool(motors_config.getNode("use_brake").real())
        self.servo_channel = int(motors_config.getNode("servo_channel").real())
        self.esc_channel = int(motors_config.getNode("esc_channel").real())
        self.max_speed = int(motors_config.getNode("max_speed").real()) / 100
        self.speed_step = motors_config.getNode("speed_step").real()
        self.angle_step = int(motors_config.getNode("angle_step").real())
        
        self.brake = False
        fs.release()

        # Init servo kit 
        self.kit = None
        timeout = 5
        while self.kit is None:
            self.logger.info('Initializing motors with PCA9685...')
            try:
                self.kit = ServoKit(channels=16)
                self.kit.servo[self.servo_channel].angle = 90
                self.kit.continuous_servo[self.esc_channel].throttle = 0
            except Exception as e:
                self.kit = None
                self.logger.error(f"Failed to initialize motors: {e}")
                self.logger.error(f"Retrying in {timeout} seconds...")
                sleep(timeout)
                timeout *= 2
        sleep(0.5)  # ensure IMU is initialized

        self.current_angle = 90
        self.target_angle = 90
        self.current_speed = 0
        self.target_speed = 0

        # Init subscribers
        self.motors_subscriber = self.create_subscription(Twist, topic, self.motors_callback, 10)
        if self.use_brake:
            self.brake_subscriber = self.create_subscription(Bool, brake_topic, self.brake_callback, 10)

        self.logger.info('Motor listener node launched.')

    def motors_callback(self, msg: Twist):
        self.logger.info("Received motor data...", once=True)

        if self.brake:
            return

        angle = msg.angular.z * RAD_TO_DEG

        # Convert from rad to degrees
        self.target_angle = angle
        self.target_angle = max(0, min(180, self.target_angle))

        self.target_speed = msg.linear.x
        self.target_speed = max(-0.7, min(0.7, self.target_speed))

    def brake_callback(self, msg: Bool):
        self.logger.info("Received brake signal...")

        if msg.data:
            self.logger.info("Braking...")
            self.kit.continuous_servo[self.esc_channel].throttle = 0
            self.kit.servo[self.servo_channel].angle = 90
            self.current_speed = 0
            self.current_angle = 90

    def make_simple_profile(self, target_speed, current_speed, step):
        print(target_speed, current_speed, step)
        if current_speed > target_speed:
            return current_speed - step
        elif current_speed < target_speed:
            return current_speed + step
        return current_speed

    def speed_profile(self):

        try: 
            while not self.brake and rclpy.ok():
                print("speed")
                self.current_speed = self.make_simple_profile(self.target_speed, self.current_speed, self.speed_step)
                self.kit.continuous_servo[self.esc_channel].throttle = self.current_speed 

                print("angle")
                self.current_angle = self.make_simple_profile(self.target_angle, self.current_angle, self.angle_step)
                self.kit.servo[self.servo_channel].angle = max(0, min(180, self.current_angle - ZERO_ANGLE_CORRECTION))
                
                print(self.current_angle, self.current_speed)

                sleep(0.001)
                rclpy.spin_once(self, timeout_sec=0.05)
        except KeyboardInterrupt:
            self.kit.servo[self.servo_channel].angle = 90 - ZERO_ANGLE_CORRECTION
            self.kit.continuous_servo[self.esc_channel].throttle = 0
            print("Keyboard interrupt detected.")
            return

    def __del__(self):
        self.kit.continuous_servo[self.esc_channel].throttle = 0
        self.kit.servo[self.servo_channel].angle = 90

if __name__ == "__main__":
    rclpy.init(args=None)

    motors_listener = MotorsListener()
    motors_listener.speed_profile()

    motors_listener.destroy_node()
    rclpy.shutdown()
