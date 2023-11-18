import rclpy
import yaml
from sensor_msgs.msg import Range
from sys import argv
from adafruit_vl53l0x import VL53L0X
import adafruit_bitbangio as bitbangio
import board
from time import sleep

def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["sensors"]["distance"]["node"]
    topic = config["sensors"]["distance"]["topic"]
    sample_rate = config["sensors"]["distance"]["sample_rate"]

    # ros2 initialization
    rclpy.init(args=argv)
    global node
    node = rclpy.create_node(node_name)
    pub = node.create_publisher(Range, topic, 10)
    rate = node.create_rate(sample_rate)  # frequency in Hz
    logger = node.get_logger()
    logger.info('Distance sensor node launched.')

    # sensor initialization
    i2c = bitbangio.I2C(scl=board.D27, sda=board.D22, frequency=sample_rate*1000)
    vl5 = VL53L0X(i2c, address=0x29)

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