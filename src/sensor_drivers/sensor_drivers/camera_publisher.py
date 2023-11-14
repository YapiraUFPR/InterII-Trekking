import rclpy
import yaml
# from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time

def main():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["sensors"]["camera"]["node"]
    topic = config["sensors"]["camera"]["topic"]
    sample_rate = config["sensors"]["camera"]["sample_rate"]
    input_stream = config["sensors"]["camera"]["input_stream"]
    resolution = config["sensors"]["camera"]["resolution"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    camera_pub = node.create_publisher(Image, topic, 10)
    rate = node.create_rate(sample_rate) # frequency in Hz
    logger = node.get_logger()
    logger.info('Camera node launched.')

    # camera initialization
    cap = cv2.VideoCapture(input_stream)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution["height"])
    cap.set(cv2.CAP_PROP_FPS, sample_rate)

    # main loop
    logger.info('Publishing frame data...')
    ret, frame = cap.read()
    while ret:
        msg = Image()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * msg.width
        msg.data = frame.tobytes()

        camera_pub.publish(msg)

        time.sleep(1/90)

        ret, frame = cap.read()


if __name__ == "__main__":
    main()