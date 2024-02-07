#!/usr/bin/env python3.7
import rclpy
import yaml
from std_msgs.msg import ColorRGBA
import board
from adafruit_tcs34725 import TCS34725
import adafruit_bitbangio as bitbangio
# import busio
from time import sleep

def main():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["sensors"]["color"]["node"]
    topic = config["sensors"]["color"]["topic"]
    sample_rate = config["sensors"]["color"]["sample_rate"]

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
        logger.info('Initializing sensor TCS347...')
        try:
            i2c = bitbangio.I2C(scl=board.D27, sda=board.D22, frequency=sample_rate*1000)
            tcs = TCS34725(i2c, address=0x29)
            tcs.gain = 16
            tcs.integration_time = 200
        except Exception as e:
            tcs = None
            timeout *= 2
            logger.error(f"Failed to initialize TCS347: {e}")
            logger.error(f"Retrying in {timeout} seconds...")
            sleep(timeout)
            timeout = 5

    # main loop
    logger.info('Publishing color data...')
    while rclpy.ok():
        
        try:
            color_bytes = tcs.color_rgb_bytes
            msg = ColorRGBA()
            msg.r = float(color_bytes[0])
            msg.g = float(color_bytes[1])
            msg.b = float(color_bytes[2])
            msg.a = 1.0

            color_pub.publish(msg)

        except Exception:
            logger.warning("Color sensor error, reseting...")
            tcs = TCS34725(i2c, address=0x29)
            sleep(3)

        sleep(1/sample_rate)


if __name__ == "__main__":
    main()