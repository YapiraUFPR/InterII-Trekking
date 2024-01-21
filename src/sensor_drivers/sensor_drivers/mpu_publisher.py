#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
# Modified by Gabriel Pontarolo, 2023

import rclpy
import yaml
from sensor_msgs.msg import Imu
import board
import busio
from time import sleep
from adafruit_mpu6050 import MPU6050

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
    imu_pub = node.create_publisher(Imu, topic, 10)
    rate = node.create_rate(sample_rate)  # frequency in Hz
    logger = node.get_logger()
    logger.info('Imu node launched.')

    # sensor initialization
    mpu = None
    timeout = 5
    while mpu is None:
        logger.info('Initializing sensor TCS347...')
        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=sample_rate*1000)
            mpu = MPU6050(i2c)
        except Exception as e:
            mpu = None
            logger.error(f"Failed to initialize TCS347: {e}")
            logger.error(f"Retrying in {timeout} seconds...")
            sleep(timeout)
            timeout *= 2
    sleep(0.5)  # ensure IMU is initialized

    # main loop
    logger.info("Publishing IMU data...")
    while True:
        imu_msg = Imu()
        imu_msg.header.stamp = node.get_clock().now().to_msg()

        gyro_x, gyro_y, gyro_z = mpu.gyro
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.angular_velocity_covariance[0] = -1

        accel_x, accel_y, accel_z = mpu.acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.linear_acceleration_covariance[0] = -1

        imu_pub.publish(imu_msg)

        sleep(1/sample_rate)


if __name__ == '__main__':
    main()