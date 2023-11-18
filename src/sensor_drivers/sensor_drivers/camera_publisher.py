import rclpy
import yaml
from sensor_msgs.msg import CompressedImage
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
    camera_pub = node.create_publisher(CompressedImage, topic, 10)
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
        msg = CompressedImage()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()

        camera_pub.publish(msg)

        time.sleep(1/sample_rate)

        ret, frame = cap.read()


if __name__ == "__main__":
    main()