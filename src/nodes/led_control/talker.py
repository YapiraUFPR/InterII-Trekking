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
from time import time

TCS_ADDR = 0x29
LED_PIN = board.D4

MARK_COLOR_LOWER = [200, 200, 200]
MARK_COLOR_UPPER = [255, 255, 255]

LED_DELAY = 0.5

led = None

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
    tcs.integration_time = 150  # time to read the signal, between 2.4 and 614.4 milliseconds
    tcs.gain = 16 # ampplification factor of the light, 1, 4, 16, 60

    # led initialization
    global led
    led = DigitalInOut(LED_PIN)

    color_msg = ColorRGBA()

    led_countdown = 0
    while rclpy.ok():

        r, g, b = tcs.color_rgb_bytes
        temp = tcs.color_temperature
        lux = tcs.lux
        
        color_msg.r = r
        color_msg.g = g
        color_msg.b = b

        color = [r, g, b]

        if all(MARK_COLOR_LOWER[i] < color[i] < MARK_COLOR_UPPER[i] for i in range(3)):
            led.value = True
            led_countdown = time() + LED_DELAY

        if time() > led_countdown:
            led.value = False

        color_pub.publish(color_msg)

try: 
    main()
except Exception:
    if led != None:
        led.value = False