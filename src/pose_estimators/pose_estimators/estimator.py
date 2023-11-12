import rclpy
import yaml
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped


def main():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["estimator"]["node"]
    topic = config["estimator"]["topic"]
    sample_rate = config["estimator"]["sample_rate"]
    vo_topic = config["vo"]["topic"]
    slam_topic = config["imu"]["topic"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    rate = node.create_rate(sample_rate) # frequency in Hz
    rate
    logger = node.get_logger()
    logger.info('Estimator node launched.')



if __name__ == "__main__":
    main()