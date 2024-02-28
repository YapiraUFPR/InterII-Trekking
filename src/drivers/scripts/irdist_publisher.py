#!/usr/bin/env python3.7
import rclpy
import yaml
from sensor_msgs.msg import Range
from sys import argv
from .libs.adafruit_vl53l0x import VL53L0X
from .libs.i2c import I2C
import board
from time import sleep

def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["sensors"]["distance"]["node"]
    topic = config["sensors"]["distance"]["topic"]
    i2c_bus = config["sensors"]["distance"]["bus"]
    sample_rate = config["sensors"]["distance"]["sample_rate"]

    # ros2 initialization
    rclpy.init(args=argv)
    global node
    node = rclpy.create_node(node_name)
    pub = node.create_publisher(Range, topic, 10)
    logger = node.get_logger()
    logger.info('Distance sensor node launched.')

    # sensor initialization
    vl5 = None
    timeout = 5
    while vl5 is None:
        logger.info('Initializing sensor TCS347...')
        try:
            i2c = I2C(i2c_bus, sample_rate*1000)
            vl5 = VL53L0X(i2c, address=0x29)
        except Exception as e:
            vl5 = None
            logger.error(f"Failed to initialize VL53L0X: {e}")
            logger.error(f"Retrying in {timeout} seconds...")
            sleep(timeout)
            timeout *= 2

    # main loop
    logger.info('Publishing distance data...')
    while True:
        msg = Range()
        msg.header.stamp = node.get_clock().now().to_msg()

        msg.range = vl5.range

        pub.publish(msg)

        sleep(1/sample_rate)

if __name__ == "__main__":
    main()