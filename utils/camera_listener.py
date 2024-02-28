import rclpy
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
from sys import argv 

topic_name = ""

def image_callback(msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    cv2.imshow("Image window", image_np)
    cv2.waitKey(1)

def main():

    global topic_name
    topic_name = argv[1]

    rclpy.init()
    global node 
    node = rclpy.create_node("camera_listener")
    image_listener = node.create_subscription(CompressedImage, topic_name, image_callback, 10)
    rclpy.spin(node)

if __name__ == "__main__":
    main()