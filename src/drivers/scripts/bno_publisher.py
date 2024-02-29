#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
# Modified by Gabriel Pontarolo, 2023

import rclpy
from rclpy.node import Node
import yaml
from sensor_msgs.msg import Imu
import board
from drivers.libs.i2c import I2C
from drivers.libs.adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    # BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from drivers.libs.adafruit_bno08x.i2c import BNO08X_I2C
from time import sleep
import cv2

class BnoPublisher(Node):

    def __init__(self):
        super().__init__('bno_publisher')

        # load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        imu_config = fs.getNode("sensors").getNode("imu")
        topic = imu_config.getNode("topic").string()
        sample_rate = int(imu_config.getNode("sample_rate").real())
        i2c_bus = int(imu_config.getNode("topic").real())
        fs.release()

        # ros2 initialization
        self.imu_pub = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(1/(sample_rate), self.timer_callback)
        self.logger = self.get_logger()
        self.logger.info('Imu node launched.')

        # sensor initialization
        self.bno = None
        timeout = 5
        while self.bno is None:
            self.logger.info('Initializing sensor BNO008x...')
            try:
                i2c = I2C(i2c_bus, sample_rate*1000)
                self.bno = BNO08X_I2C(i2c, address=0x4b)  # BNO080 (0x4b) BNO085 (0x4a)
                self.bno.initialize()
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            except Exception as e:
                self.bno = None
                self.logger.error(f"Failed to initialize BNO008x: {e}")
                self.logger.error(f"Retrying in {timeout} seconds...")
                sleep(timeout)
                timeout *= 2
        sleep(0.5)  # ensure IMU is initialized


    def timer_callback(self):
        self.logger.info("Publishing IMU data...", once=True)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu"

        # quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        # imu_msg.orientation.x = quat_i
        # imu_msg.orientation.y = quat_j
        # imu_msg.orientation.z = quat_k
        # imu_msg.orientation.w = quat_real
        # imu_msg.orientation_covariance[0] = -1

        # gyro_x, gyro_y, gyro_z = self.bno.gyro
        # imu_msg.angular_velocity.x = gyro_x
        # imu_msg.angular_velocity.y = gyro_y
        # imu_msg.angular_velocity.z = gyro_z
        # imu_msg.angular_velocity_covariance[0] = -1

        accel_x, accel_y, accel_z = self.bno.acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.linear_acceleration_covariance[0] = -1
        
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    bno_publisher = BnoPublisher()

    rclpy.spin(bno_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bno_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# def main():

#     # load config
#     with open("/home/user/ws/src/config/config.yaml", "r") as file:
#         config = yaml.safe_load(file)
#     topic = config["sensors"]["imu"]["topic"]
#     sample_rate = config["sensors"]["imu"]["sample_rate"]
#     i2c_bus = config["sensors"]["imu"]["bus"]

#     # ros2 initialization
#     rclpy.init(args=None)
#     global node
#     node = rclpy.create_node("bno08x")
#     imu_pub = node.create_publisher(Imu, topic, 10)
#     logger = node.get_logger()
#     logger.info('Imu node launched.')

#     # sensor initialization
#     bno = None
#     timeout = 5
#     while bno is None:
#         logger.info('Initializing sensor BNO008x...')
#         try:
#             i2c = I2C(board.SCL, board.SDA, sample_rate*1000)
#             bno = BNO08X_I2C(i2c, address=0x4b)  # BNO080 (0x4b) BNO085 (0x4a)
#             bno.initialize()
#             bno.enable_feature(BNO_REPORT_ACCELEROMETER)
#             bno.enable_feature(BNO_REPORT_GYROSCOPE)
#             bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
#         except Exception as e:
#             bno = None
#             logger.error(f"Failed to initialize BNO008x: {e}")
#             logger.error(f"Retrying in {timeout} seconds...")
#             sleep(timeout)
#             timeout *= 2
#     sleep(0.5)  # ensure IMU is initialized

#     # main loop
#     logger.info("Publishing IMU data...")
#     while rclpy.ok():
#         imu_msg = Imu()
#         imu_msg.header.stamp = node.get_clock().now().to_msg()
#         imu_msg.header.frame_id = "imu"

#         quat_i, quat_j, quat_k, quat_real = bno.quaternion
#         imu_msg.orientation.x = quat_i
#         imu_msg.orientation.y = quat_j
#         imu_msg.orientation.z = quat_k
#         imu_msg.orientation.w = quat_real
#         imu_msg.orientation_covariance[0] = -1

#         gyro_x, gyro_y, gyro_z = bno.gyro
#         imu_msg.angular_velocity.x = gyro_x
#         imu_msg.angular_velocity.y = gyro_y
#         imu_msg.angular_velocity.z = gyro_z
#         imu_msg.angular_velocity_covariance[0] = -1

#         accel_x, accel_y, accel_z = bno.acceleration
#         imu_msg.linear_acceleration.x = accel_x
#         imu_msg.linear_acceleration.y = accel_y
#         imu_msg.linear_acceleration.z = accel_z
#         imu_msg.linear_acceleration_covariance[0] = -1
        
#         imu_pub.publish(imu_msg)

#         # sleep(1/sample_rate)