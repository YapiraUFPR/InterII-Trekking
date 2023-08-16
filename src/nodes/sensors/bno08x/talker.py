#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
# Modified by Gabriel Pontarolo, 2023

import rclpy
from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticStatus
import time
import sys
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

def bno08x_node():
    rclpy.init(args=sys.argv)

    global node
    node = rclpy.create_node('bno08x')

    raw_pub = node.create_publisher(Imu, 'bno08x/raw', 10)
    mag_pub = node.create_publisher(MagneticField, 'bno08x/mag', 10)
    status_pub = node.create_publisher(DiagnosticStatus, 'bno08x/status', 10)

    rate = node.create_rate(100)  # frequency in Hz
    node.get_logger().info('bno08x node launched.')

    i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
    bno = BNO08X_I2C(i2c, address=0x4b)  # BNO080 (0x4b) BNO085 (0x4a)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    time.sleep(0.5)  # ensure IMU is initialized

    while rclpy.ok():
        raw_msg = Imu()
        raw_msg.header.stamp = node.get_clock().now().to_msg()

        accel_x, accel_y, accel_z = bno.acceleration
        raw_msg.linear_acceleration.x = accel_x
        raw_msg.linear_acceleration.y = accel_y
        raw_msg.linear_acceleration.z = accel_z

        gyro_x, gyro_y, gyro_z = bno.gyro
        raw_msg.angular_velocity.x = gyro_x
        raw_msg.angular_velocity.y = gyro_y
        raw_msg.angular_velocity.z = gyro_z

        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        raw_msg.orientation.w = quat_i
        raw_msg.orientation.x = quat_j
        raw_msg.orientation.y = quat_k
        raw_msg.orientation.z = quat_real

        raw_msg.orientation_covariance[0] = -1
        raw_msg.linear_acceleration_covariance[0] = -1
        raw_msg.angular_velocity_covariance[0] = -1

        raw_pub.publish(raw_msg)

        mag_msg = MagneticField()
        mag_x, mag_y, mag_z = bno.magnetic
        mag_msg.header.stamp = node.get_clock().now().to_msg()
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z
        mag_msg.magnetic_field_covariance[0] = -1
        mag_pub.publish(mag_msg)

        status_msg = DiagnosticStatus()
        status_msg.level = 0
        status_msg.name = "bno08x IMU"
        status_msg.message = ""
        status_pub.publish(status_msg)

        rate.sleep()

    node.get_logger().info('bno08x node finished')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        bno08x_node()
    except Exception:
        node.get_logger().info('bno08x node exited with exception')