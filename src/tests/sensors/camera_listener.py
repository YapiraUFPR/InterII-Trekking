import rclpy
import yaml
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

def camera_callback(msg):
    global node
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imshow("frame", image_np)
    cv2.waitKey(1)

def camera_pub():
    
    # load config
    with open("/home/gab/projetos/yapira/bedman-trekker/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    # camera_topic = config["sensors"]["camera"]["topic"]
    camera_topic = "/optflow"

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node("camera_listener")
    camera_sub = node.create_subscription(CompressedImage, camera_topic, camera_callback, 10)
    rate = node.create_rate(10) # frequency in Hz
    camera_sub, rate
    logger = node.get_logger()
    logger.info('Camera listener node launched.')

    rclpy.spin(node)


if __name__ == "__main__":
    camera_pub()