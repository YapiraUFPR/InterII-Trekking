#!/usr/bin/env python3

# Node to control the LED strip using the TCS34725 color sensor
# LED strip should light up when color sensor detects the mark color
# Author: Gabriel Pontarolo

import rclpy
from adafruit_tcs34725 import TCS34725
import board
import busio
from digitalio import DigitalInOut 
from std_msgs.msg import ColorRGBA
from sys import argv
from time import sleep

MARK_COLOR = [255, 255, 255]
TCS_ADDR = 0x29
LED_PIN = board.D13

def main():

    rclpy.init(args=argv)

    # node intialization
    global node 
    node = rclpy.create_node('led_control')
    color_pub = node.create_publisher(ColorRGBA, 'led_control/color', 10)
    rate = node.create_rate(100) # frequency in Hz
    node.get_logger().info('led_control node launched.')

    # sensor initialization
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    tcs = TCS34725(i2c,address=TCS_ADDR) #0x29

    # led initialization
    led_pin = DigitalInOut(LED_PIN)

    color_msg = ColorRGBA()

    led_on = False
    while rclpy.ok():

        r, g, b = tcs.color_rgb_bytes
        temp = tcs.color_temperature
        lux = tcs.lux

        color_msg.r = r
        color_msg.g = g
        color_msg.b = b

        if MARK_COLOR == [r, g, b] and not led_on:
            led_pin.value = True
            led_on = True
        elif MARK_COLOR != [r, g, b] and led_on:
            led_pin.value = False
            led_on = False

        color_pub.publish(color_msg)



if __name__ == "__main__":
    main()