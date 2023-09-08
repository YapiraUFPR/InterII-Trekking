import rclpy
from sensor_msgs.msg import Range
from sys import argv
from adafruit_vl53l0x import VL53L0X
import adafruit_bitbangio as bitbangio
import board

SAMPLE_RATE = 400 

def vl53l0x_node():
    rclpy.init(args=argv)

    global node
    node = rclpy.create_node('vl53l0x')
    pub = node.create_publisher(Range, 'vl53l0x/raw', 10)
    rate = node.create_rate(100)  # frequency in Hz
    node.get_logger().info('vl53l0x node launched.')

    i2c = bitbangio.I2C(scl=board.D5, sda=board.D6, frequency=SAMPLE_RATE*1000)
    vl5 = VL53L0X(i2c, address=0x29)

    while True:
        msg = Range()
        msg.header.stamp = node.get_clock().now().to_msg()

        msg.range = vl5.range

        pub.publish(msg)

        rate.sleep()