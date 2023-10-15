import rclpy
import yaml
from std_msgs.msg import ColorRGBA
import board
from adafruit_tcs34725 import TCS34725
import adafruit_bitbangio as bitbangio

def color_pub():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["color"]["node"]
    topic = config["color"]["topic"]
    sample_rate = config["color"]["sample_rate"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    color_pub = node.create_publisher(ColorRGBA, topic, 10)
    rate = node.create_rate(sample_rate) # frequency in Hz
    logger = node.get_logger()
    logger.info('Color node launched.')

    # sensor initialization, uses secondary i2c bus to avoid conflicts with distance sensor
    i2c = bitbangio.I2C(scl=board.D27, sda=board.D22, frequency=sample_rate*1000)
    tcs = TCS34725(i2c, address=0x29)

    # main loop
    logger.info('Publishing color data...')
    while True:
        msg = ColorRGBA()
        msg.r, msg.g, msg.b = tcs.color_rgb_bytes
        msg.a = 1.0

        color_pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    color_pub()