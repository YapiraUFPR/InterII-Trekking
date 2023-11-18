import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import numpy as np
import yaml

KP = 0.5
KI = 0.1
KD = 0.1

last_error = 0

def point_callback(msg):
    global node
    global last_error

    node.get_logger().info(f"Received point msg {msg}")

    x = msg.x
    y = msg.y

    error = x - 0.5
    integral = error
    derivative = error - last_error
    last_error = error

    pid = KP*error + KI*integral + KD*derivative

    twist = Twist()
    twist.linear.x = 0.5
    twist.angular.z = pid

    node.get_logger().info(f"Publishing twist msg {twist}")
    twist_publisher.publish(twist)

def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
        node_name = config["pid"]["node"]
        topic = config["pid"]["topic"]
        sample_rate = config["pid"]["sample_rate"]
        point_topic = config["cone_detector"]["topic"]









