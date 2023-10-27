import rclpy
import yaml
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from monocular_visual_odometry import MonocularVisualOdometry

image_buffer = []

def camera_callback(msg):
    global node
    global image_buffer
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
    image_buffer.append(gray)

def odometry():
    
    # load config
    with open("/home/gab/projetos/yapira/bedman-trekker/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    camera_topic = config["sensors"]["camera"]["topic"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node("camera_listener")
    camera_sub = node.create_subscription(CompressedImage, camera_topic, camera_callback, 10)
    rate = node.create_rate(10) # frequency in Hz
    camera_sub, rate
    logger = node.get_logger()
    logger.info('Camera listener node launched.')

    # get first frame
    rclpy.spin_once(node)

    global image_buffer
    first_frame = image_buffer.pop(0)
    vo = MonocularVisualOdometry(first_frame)

    while True:
        rclpy.spin_once(node)

        if len(image_buffer) > 0:
            curr_frame = image_buffer.pop(0)
            R, t = vo.estimate(curr_frame)
            # print(R, t)

if __name__ == "__main__":
    odometry()