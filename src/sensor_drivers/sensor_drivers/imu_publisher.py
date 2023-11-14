#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
# Modified by Gabriel Pontarolo, 2023

import rclpy
import yaml
from custom_messages.msg import Imu9DOF
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from time import sleep

def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["sensors"]["imu"]["node"]
    topic = config["sensors"]["imu"]["topic"]
    sample_rate = config["sensors"]["imu"]["sample_rate"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    imu_pub = node.create_publisher(Imu9DOF, topic, 10)
    rate = node.create_rate(sample_rate)  # frequency in Hz
    logger = node.get_logger()
    logger.info('Imu node launched.')

    # sensor initialization 
    i2c = busio.I2C(board.SCL, board.SDA, frequency=sample_rate*1000)
    bno = BNO08X_I2C(i2c, address=0x4b)  # BNO080 (0x4b) BNO085 (0x4a)
    bno.initialize()
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    sleep(0.5)  # ensure IMU is initialized

    # main loop
    logger.info("Publishing IMU data...")
    while True:
        imu_msg = Imu9DOF()
        imu_msg.header.stamp = node.get_clock().now().to_msg()

        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        imu_msg.orientation.x = quat_i
        imu_msg.orientation.y = quat_j
        imu_msg.orientation.z = quat_k
        imu_msg.orientation.w = quat_real
        imu_msg.orientation_covariance[0] = -1

        gyro_x, gyro_y, gyro_z = bno.gyro
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.angular_velocity_covariance[0] = -1

        accel_x, accel_y, accel_z = bno.acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.linear_acceleration_covariance[0] = -1

        mag_x, mag_y, mag_z = bno.magnetic
        imu_msg.magnetic_field.x = mag_x
        imu_msg.magnetic_field.y = mag_y
        imu_msg.magnetic_field.z = mag_z
        imu_msg.magnetic_field_covariance[0] = -1
        
        imu_pub.publish(imu_msg)

        rate.sleep()


if __name__ == '__main__':
    main()