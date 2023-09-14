from libs.IMU_tracker import IMUTracker
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sys import argv
import numpy as np
import time

CALIB_DATA_SIZE = 1000

finished_callib = False
imu_data = []
mag_data = []

def imu_callback(msg:Imu):

    global imu_data
    imu_data = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, 
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

def mag_callback(msg:MagneticField):

    global mag_data
    mag_data = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]

def position_tracker():
    rclpy.init(args=argv)

    # node initialization
    global node
    node = rclpy.create_node('position_tracker')
    imu_sub = node.create_subscription(Imu, 'bno08x/raw', imu_callback, 10)
    mag_sub = node.create_subscription(MagneticField, 'bno08x/mag', mag_callback, 10)
    pos_pub = node.create_publisher(PoseStamped, 'position', 10)
    rate = node.create_rate(100)  # frequency in Hz
    imu_sub, mag_sub, pos_pub, rate  # prevent unused variable warning
    node.get_logger().info('position_tracker node launched.')

    tracker = IMUTracker(sampling=400)
    
    global finished_callib
    global imu_data
    global mag_data

    node.get_logger().info('waiting for callibration data...')
    callib_data = []
    while not finished_callib:
        rclpy.spin_once(node)

        if len(imu_data) > 0 and len(mag_data) > 0:
            callib_data.append(imu_data + mag_data)
            imu_data = []
            mag_data = []

        if len(callib_data) > CALIB_DATA_SIZE:
            finished_callib = True

        time.sleep(1/400)


    callib_data_np = np.array(callib_data, dtype=np.float32)
    print(callib_data_np.shape)
    tracker.initialize(callib_data_np)
    node.get_logger().info('callibration finished.')

    node.get_logger().info('position tracking started.')
    i = 0
    while True:
        rclpy.spin_once(node)

        if len(imu_data) > 0 and len(mag_data) > 0:
            data = []
            data += imu_data + mag_data
            imu_data = []
            mag_data = []
            data_np = np.array(data, dtype=np.float32)

            p = tracker.calculatePosition(data_np)

            # Publish position
            msg = PoseStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x = p[0]
            msg.pose.position.y = p[1]
            msg.pose.position.z = p[2]
            pos_pub.publish(msg)

            i += 1

        time.sleep(1/400)

if __name__ == '__main__':
    position_tracker()
