#!/usr/bin/env python3

# Node to create a map of the track
# Author: Vinicius Ribeiro

import rclpy
import numpy as np
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import Imu
import pickle

x_positions = np.array([])
y_positions = np.array([])

x_current_velocity = 0
y_current_velocity = 0

x_current_position = 0
y_current_position = 0

prev_timestamp = None


def save_trajectory():
    global x_positions
    global y_positions

    spline = CubicSpline(x_positions, y_positions)

    # Save the CubicSpline object to a file using pickle
    with open('cubic_spline.pkl', 'wb') as f:
        pickle.dump(spline, f)


def imu_callback(msg):
    global x_positions
    global y_positions
    global x_current_velocity
    global y_current_velocity
    global x_current_position
    global y_current_position
    global prev_timestamp

    if prev_timestamp is None:
        prev_timestamp = msg.header.stamp
        return

    delta_t = (msg.header.stamp - prev_timestamp).to_sec()
    prev_timestamp = msg.header.stamp

    # Integrate acceleration to get velocity
    x_current_velocity += msg.linear_acceleration.x * delta_t
    y_current_velocity += msg.linear_acceleration.y * delta_t

    # Integrate velocity to get position
    x_current_position += x_current_velocity * delta_t
    y_current_position += y_current_velocity * delta_t

    x_positions = np.append(x_positions, x_current_position)
    y_positions = np.append(y_positions, y_current_position)

    save_trajectory()

def main():

    rclpy.init(args=argv)
    
    # node intialization
    node = rclpy.create_node('carlos')
    imu_sub = node.create_subscription(Imu, 'bno08x/raw', imu_callback, 10)
    node.get_logger().info('carlos node launched.')



if __name__ == '__main__':
    main()