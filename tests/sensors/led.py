#!/usr/bin/env python3

# Node to control the LED strip using the TCS34725 color sensor
# LED strip should light up when color sensor detects the mark color
# Author: Gabriel Pontarolo

# import rclpy
from adafruit_tcs34725 import TCS34725
import board
import busio
from digitalio import DigitalInOut 
# from std_msgs.msg import ColorRGBA
from sys import argv
from time import time
import adafruit_bitbangio as bitbangio

TCS_ADDR = 0x29
LED_PIN = board.D4

MARK_COLOR_LOWER = [22, 22, 0]
MARK_COLOR_UPPER = [30, 30, 5]

LED_DELAY = 0.5
SAMPLE_RATE = 400

led = None

def main():

    # rclpy.init(args=argv)

    # # node intialization
    # global node 
    # node = rclpy.create_node('led_control')
    # color_pub = node.create_publisher(ColorRGBA, 'led_control/color', 10)
    # rate = node.create_rate(100) # frequency in Hz
    # node.get_logger().info('led_control node launched.')

    # sensor initialization
    tcs_i2c = bitbangio.I2C(scl=board.D27, sda=board.D22, frequency=SAMPLE_RATE*1000)
    tcs = TCS34725(tcs_i2c,address=0x29) #0x29
    tcs.integration_time = 50  # time to read the signal, between 2.4 and 614.4 milliseconds
    tcs.gain = 16 # ampplification factor of the light, 1, 4, 16, 60

    # led initialization
    global led
    led = DigitalInOut(LED_PIN)
    led.switch_to_output(value=False)

    # color_msg = ColorRGBA()

    led_countdown = 0
    while True:

        r, g, b = tcs.color_rgb_bytes
        temp = tcs.color_temperature
        lux = tcs.lux
        
        # color_msg.r = r
        # color_msg.g = g
        # color_msg.b = b

        color = [r, g, b]
        print(color,  lux, temp)

        if all(MARK_COLOR_LOWER[i] < color[i] < MARK_COLOR_UPPER[i] for i in range(3)):
            led.value = True
            led_countdown = time() + LED_DELAY

        if time() > led_countdown:
            led.value = False

        # color_pub.publish(color_msg)

try: 
    main()
except Exception as e:
    print(e)
    if led != None:
        led.value = False