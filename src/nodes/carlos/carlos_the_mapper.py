#!/usr/bin/env python3

# Node to create a map of the track
# Author: Vinicius Ribeiro

import rclpy
import numpy as np
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import pickle
from sys import argv

points = np.array([])
raw_points = np.array([])

def save_trajectory():
    global x_positions
    global y_positions

    spline = CubicSpline(x_positions, y_positions)

    # Save the CubicSpline object to a file using pickle
    with open('cubic_spline.pkl', 'wb') as f:
        pickle.dump(spline, f)

def position_callback(msg):
    global points
    x, y, z = msg.pose.position
    np.append(points, [x, y])

def imu_callback(msg):
    global raw_points
    x, y, z = msg.linear_acceleration
    np.append(raw_points, [x, y])

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
except KeyboardInterrupt:
    np.save('points.npy', points)
    np.save('raw_points.npy', raw_points)
