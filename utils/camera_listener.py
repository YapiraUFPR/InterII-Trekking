import rclpy
import cv2
import cv_bridge

from sensor_msgs.msg import Image
import numpy as np

from sys import argv 

bridge = cv_bridge.CvBridge()

def image_callback(msg):
    print("Received image")
    image_np = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    cv2.imshow("Image window", image_np)
    cv2.waitKey(5)

def main():

    topic_name = argv[1]

    rclpy.init()
    global node 
    node = rclpy.create_node("camera_listener")
    image_listener = node.create_subscription(Image, topic_name, image_callback, 10)
    image_listener  # prevent unused variable warning
    rclpy.spin(node)

if __name__ == "__main__":
    main()