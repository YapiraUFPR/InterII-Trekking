import rclpy
import cv2
from nav_msgs.msg import Odometry
import numpy as np
from threading import Lock
from sys import argv 

OFFSET = 250
SCALE = 1

track_image = np.zeros((500, 500, 3), np.uint8)
cv2.line(track_image, (0, OFFSET), (OFFSET*2, OFFSET), (255, 255, 255)) # x-axis
cv2.line(track_image, (OFFSET, 0), (OFFSET, OFFSET*2), (255, 255, 255)) # y-axis

track_image_lock = Lock()

prev_pos = (0,0)
prev_map = (0,0)

def draw_line(start, end, color=(0, 255, 0)):
    global track_image
    global track_image_lock

    print("Drawing...")

    start = (int(start[0] * SCALE + OFFSET), int(start[1] * SCALE + OFFSET))
    end = (int(end[0] * SCALE + OFFSET), int(end[1] * SCALE + OFFSET))
    with track_image_lock:
        cv2.line(track_image, start, end, color, 2)

    cv2.imshow("Track", track_image)
    cv2.waitKey(1)

def pos_callback(msg):
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
    global prev_pos
    draw_line(prev_pos, (x, y), (0, 255, 0))
    prev_pos = (x, y)

def map_callback(msg):
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
    global prev_map
    draw_line(prev_map, (x, y), (0, 0, 255))
    prev_map = (x, y)


def main():

    curr_pos_topic = argv[1]
    map_topic = argv[2]

    rclpy.init()
    global node 
    node = rclpy.create_node("track_drawer")
    position_listener = node.create_subscription(Odometry, curr_pos_topic, pos_callback, 10)
    map_listener = node.create_subscription(Odometry, map_topic, map_callback, 10)
    position_listener, map_listener
    rclpy.spin(node)

if __name__ == "__main__":
    main()