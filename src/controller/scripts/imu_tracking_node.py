#!/usr/bin/env python3

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import sleep
import numpy as np

from custom_msgs.msg import Imu
from nav_msgs.msg import Odometry

from controller.imu_position_tracking.imu_tracker import IMUTracker

class ImuTrackerNode(Node):

    def __init__(self):
        super().__init__('imu_tracker_node')
        self.logger = self.get_logger()
        self.logger.info('Initializing IMU position tracker node...')

        # Load config
        # fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        imu_topic = "/sensors/bno08x/raw"
        topic = "/imu_tracker/odom"
        self.sample_rate = 150
        # fs.release()

        # Init subscribers
        self.imu_subscriber = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, topic, 10)

        self.imu_tracker = IMUTracker(sampling=self.sample_rate)
        self.initialized = False
        self.imu_data = []

        self.logger.info('IMU tracker node launched.')

    def imu_callback(self, msg: Imu):
        self.logger.info("Received IMU data...", once=True)

        data = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]

        self.imu_data.append(data)

        if not self.initialized and len(self.imu_data) > 600:
            self.imu_tracker.initialize(np.array(self.imu_data[100:], dtype=np.float32))
            self.initialized = True
            self.logger.info("IMU tracker initialized.", once=True)
            self.imu_data = []

        if self.initialized and len(self.imu_data) > (self.sample_rate / 10): # Estimate position every 0.1s
            P = self.imu_tracker.track(np.array(self.imu_data, dtype=np.float32))
            self.imu_data = []

            # print("Estimation:")
            # print(P)
            # print(Q)

            msg = Odometry()
            msg.pose.pose.position.x = float(P[0])
            msg.pose.pose.position.y = float(P[1])
            msg.pose.pose.position.z = 0.0
            # msg.pose.pose.orientation.w = float(Q[0])
            # msg.pose.pose.orientation.x = float(Q[1])
            # msg.pose.pose.orientation.y = float(Q[2])
            # msg.pose.pose.orientation.z = float(Q[3])
            self.odom_publisher.publish(msg)

if __name__ == "__main__":
    rclpy.init(args=None)

    imu_tracker_node = ImuTrackerNode()
    rclpy.spin(imu_tracker_node)

    imu_tracker_node.destroy_node()
    rclpy.shutdown()