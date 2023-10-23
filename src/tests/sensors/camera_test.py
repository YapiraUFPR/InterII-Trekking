import rclpy
import yaml
from std_msgs.msg import CompressedImage
import board
#from adafruit_tcs34725 import TCS34725
#import adafruit_bitbangio as bitbangio
# Python libs
import sys, time
import numpy as np
import cv2

def camera_pub():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["camera"]["node"]
    topic = config["camera"]["topic"]
    sample_rate = config["camera"]["sample_rate"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    camera_pub = node.create_publisher(CompressedImage, topic, 10)
    rate = node.create_rate(sample_rate) # frequency in Hz
    logger = node.get_logger()
    logger.info('Camera node launched.')

    # sensor initialization, uses secondary i2c bus to avoid conflicts with distance sensor
    #i2c = bitbangio.I2C(scl=board.D27, sda=board.D22, frequency=sample_rate*1000)
    #tcs = TCS34725(i2c, address=0x29)

    # main loop
    logger.info('Publishing frame data...')
    cap = cv2.VideoCapture(0)
    while True:
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cap)[1]).tostring()

        camera_pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    camera_pub()