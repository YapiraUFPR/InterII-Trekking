#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2

from geometry_msgs.msg import Twist
from std_msgs.msg import String

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

ANG_TO_RAD = 3.14159265358979323846 / 180.0

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_bluetooth')
        self.logger = self.get_logger()
        self.logger.info('Initializing teleop node...')

        # Load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        bluetooth_topic = fs.getNode("sensors").getNode("bluetooth_gamepad").getNode("topic").string()
        motors_topic = fs.getNode("actuators").getNode("motors").getNode("topic").string()
        fs.release()

        self.bluetooth_sub = self.create_subscription(String, bluetooth_topic, self.bluetooth_callback, 10)
        self.motors_pub = self.create_publisher(Twist, motors_topic, 10)

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.target_angular_position = 0.0
        self.acceleration = LIN_VEL_STEP_SIZE

        self.logger.info('Teleop node launched.')
    
    def bluetooth_callback(self, msg):
        if msg.data == "UP_PRESSED":
            self.target_linear_velocity = self.acceleration
        elif msg.data == "DOWN_PRESSED":
            self.target_linear_velocity = -self.acceleration

        elif msg.data == "L1":
            self.acceleration -= LIN_VEL_STEP_SIZE
            # self.acceleration = max(0, min(self.acceleration, 100))
        elif msg.data == "R1":
            self.acceleration += LIN_VEL_STEP_SIZE
            # self.acceleration = max(0, min(self.acceleration, 100))

        elif msg.data == "LEFT_PRESSED":
            self.target_angular_velocity = -ANG_VEL_STEP_SIZE
        elif msg.data == "RIGHT_PRESSED":
            self.target_angular_velocity = ANG_VEL_STEP_SIZE

        elif msg.data == "UP_RELEASE" or msg.data == "DOWN_RELEASE":
            self.target_linear_velocity = 0.0
        elif msg.data == "LEFT_RELEASE" or msg.data == "RIGHT_RELEASE":
            self.target_angular_velocity = 0.0

        elif msg.data == "A":
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.target_angular_position = 0.0
            self.acceleration = LIN_VEL_STEP_SIZE

        else:
            # self.logger.warn(f"Unknown command: {msg.data}")
            return
        
        self.target_angular_position += self.target_angular_velocity
        angle = self.target_angular_position * 90 + 90
        angle = int(max(0, min(angle, 180)))
        mspeed = max(-1.0, min(self.target_linear_velocity, 1.0))

        self.logger.info(f"{angle}, {mspeed}, {self.acceleration}")

        twist = Twist()
        twist.linear.x = mspeed
        twist.angular.z = angle * ANG_TO_RAD
        self.motors_pub.publish(twist)


if __name__ == '__main__':
    rclpy.init()
    node = TeleopNode()
    rclpy.spin(node)

    rclpy.destroy_node(node)
    rclpy.shutdown()
