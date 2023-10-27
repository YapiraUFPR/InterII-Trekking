#!/usr/bin/env python3

# Node to create a map of the track
# Author: Vinicius Ribeiro

import rclpy
import numpy as np
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from sys import argv

points = []
raw_data = []

def position_callback(msg):
    global points
    x = msg.pose.position.x
    y = msg.pose.position.y 
    points.append([x, y])

def imu_callback(msg):
    global raw_data
    x = msg.linear_acceleration.x 
    y = msg.linear_acceleration.y
    raw_data.append([x, y])

def main():

    rclpy.init(args=argv)
    
    # node intialization
    node = rclpy.create_node('carlos')

    imu_sub = node.create_subscription(Imu, 'bno08x/raw', imu_callback, 10)
    pos_sub = node.create_subscription(PoseStamped, 'position', position_callback, 10)
    imu_sub, pos_sub # prevent unused variable warning
    node.get_logger().info('carlos node launched.')

    rclpy.spin(node)
try:
    main()
finally:
    points_np = np.array(points)
    raw_data_np = np.array(raw_data)
    np.save('points.npy', points_np)
    np.save('raw_data.npy', raw_data_np)
