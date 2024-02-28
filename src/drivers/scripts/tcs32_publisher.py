#!/usr/bin/env python3
import rclpy
import yaml
from std_msgs.msg import ColorRGBA
from time import sleep
from .libs.gy_tcs3200 import TCS3200

def main():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["sensors"]["color"]["node"]
    topic = config["sensors"]["color"]["topic"]
    sample_rate = config["sensors"]["color"]["sample_rate"]
    pin_s2 = config["sensors"]["color"]["pins"]["s2"]
    pin_s3 = config["sensors"]["color"]["pins"]["s3"]
    pin_signal = config["sensors"]["color"]["pins"]["signal"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    color_pub = node.create_publisher(ColorRGBA, topic, 10)
    rate = node.create_rate(sample_rate) # frequency in Hz
    logger = node.get_logger()
    logger.info('Color node launched.')

    # sensor initialization, uses secondary i2c bus to avoid conflicts with distance sensor
    tcs = None
    while tcs is None:
        logger.info('Initializing sensor TCS3200...')
        try:
            tcs = TCS3200(pin_s2, pin_s3, pin_signal)
        except Exception as e:
            tcs = None
            timeout *= 2
            logger.error(f"Failed to initialize TCS3200: {e}")
            logger.error(f"Retrying in {timeout} seconds...")
            sleep(timeout)
            timeout = 5

    # main loop
    logger.info('Publishing color data...')
    while rclpy.ok():
        
        rgb = tcs.read()
        msg = ColorRGBA()
        msg.r = (rgb[0])
        msg.g = (rgb[1])
        msg.b = (rgb[2])
        msg.a = 1.0

        color_pub.publish(msg)

        sleep(1/sample_rate)


if __name__ == "__main__":
    main()