#!/usr/bin/env python3

# Node to create a map of the track
# Author: Vinicius Ribeiro

import rclpy
import numpy as np
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from sys import argv

points = np.array([])
raw_data = np.array([])

def position_callback(msg):
    global points
    x, y, z = msg.pose.position
    np.append(points, [x, y])

def imu_callback(msg):
    global raw_data
    x, y, z = msg.linear_acceleration
    np.append(raw_data, [x, y])

def main():

    rclpy.init(args=argv)
    
    # node intialization
    node = rclpy.create_node('carlos')

    imu_sub = node.create_subscription(Imu, 'bno08x/raw', imu_callback, 10)
    pos_sub = node.create_subscription(PoseStamped, 'position', position_callback, 10)
    imu_sub, pos_sub # prevent unused variable warning
    node.get_logger().info('carlos node launched.')

try:
    main()
finally:
    np.save('points.npy', points)
    np.save('raw_data.npy', raw_data)
