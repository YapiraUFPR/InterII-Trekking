#!/usr/bin/env python3

# Node to control the LED strip using the TCS34725 color sensor
# LED strip should light up when color sensor detects the mark color
# Author: Gabriel Pontarolo

import rclpy
import yaml
import board
from digitalio import DigitalInOut 
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool
from time import time

LED_PIN = board.D10

led = None
mark_color_upper = [255, 255, 255]
mark_color_lower = [0, 0, 0]
deactivation_delay = 0.0
led_countdown = 0

def color_callback(msg:ColorRGBA):
    global led
    global mark_color_lower
    global mark_color_upper
    global deactivation_delay
    global led_countdown
    global node 
    global mark_pub

    color = [msg.r, msg.g, msg.b]
    node.get_logger().info(f"Color detected: {color}")

    if all(mark_color_lower[i] < color[i] < mark_color_upper[i] for i in range(3)):
        node.get_logger().info("Mark detected!")
        led.value = True
        led_countdown = time() + deactivation_delay
        mark_pub.publish(Bool(data=True))

    if time() > led_countdown:
        led.value = False
        mark_pub.publish(Bool(data=False))

def led_control():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    global mark_color_upper
    global mark_color_lower
    global deactivation_delay
    node_name = config["led"]["node"]
    color_topic = config["sensors"]["color"]["topic"]
    topic = config["led"]["topic"]
    mark_color_upper = config["led"]["mark_color_upper"]
    mark_color_lower = config["led"]["mark_color_lower"]
    deactivation_delay = config["led"]["deactivation_delay"]

    # ros2 initialization
    rclpy.init()
    global node 
    node = rclpy.create_node(node_name)
    color_sub = node.create_subscription(ColorRGBA, color_topic, color_callback, 10)
    global mark_pub
    mark_pub = node.create_publisher(Bool, topic, 10)
    color_sub # prevent unused variable warning
    logger = node.get_logger()
    logger.info('LED node launched.')
   
    # led initialization
    global led
    led = DigitalInOut(LED_PIN)
    led.switch_to_output(value=False)

    # main loop
    logger.info('LED node running...')
    rclpy.spin(node)

if __name__ == "__main__":
    try: 
        led_control()
    except Exception as e:
        print(e)
        # turn off led if node crashes
        if led != None:
            led.value = False