import rclpy
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
import yaml
from datetime import datetime
from os.path import isdir
from os import makedirs

MAP_SIZE = 1000
AXIS_COLOR = (0, 0, 0)
LINE_COLOR = (0, 0, 255)
TEXT_COLOR = (0, 255, 0)

position = np.zeros(2, dtype=np.float32)

map_image = None
map_points = None
output_path = None

def pose_callback(msg:PoseStamped):
    global position
    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y

def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["mapper"]["node"]
    topic = config["mapper"]["topic"]
    sample_rate = config["mapper"]["sample_rate"]
    pose_topic = config["estimator"]["topic"]
    seg_pts_num = config["mapper"]["seg_pts_num"]
    
    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    rate = node.create_rate(sample_rate) # frequency in Hz
    map_pub = node.create_publisher(CompressedImage, topic, 10)
    pose_sub = node.create_subscription(PoseStamped, pose_topic, pose_callback, 10)
    rate, map_pub, pose_sub
    logger = node.get_logger()
    logger.info('Mapper node launched.')

    # map initialization
    global output_path
    output_path = "/home/user/ws/Data/maps/"
    if not isdir(output_path):
        makedirs(output_path)
    filename = f"map_{datetime.now().strftime('%Y%m%d-%H%M%S')}.npy"
    output_path += filename
    
    # start rgb white image
    global map_image
    map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
    map_img.fill(255)
    # draw coordinate axis x and y
    cv2.line(map_img, (0, MAP_SIZE//2), (MAP_SIZE, MAP_SIZE//2), AXIS_COLOR, 2)
    cv2.line(map_img, (MAP_SIZE//2, 0), (MAP_SIZE//2, MAP_SIZE), AXIS_COLOR, 2)
    cv2.putText(map_img, "X", (MAP_SIZE - 100, (MAP_SIZE//2) + 100), cv2.FONT_HERSHEY_SIMPLEX, 2,TEXT_COLOR, 2)
    cv2.putText(map_img, "Y", ((MAP_SIZE//2) - 100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2,TEXT_COLOR, 2)

    global position
    global map_points
    pts_count = 0
    map_points = [(0, 0)]
    while True:
        rclpy.spin_once(node)
        
        if pts_count % seg_pts_num == 0:
            map_points.append((position[0], position[1]))

            pt1_int = np.array(map_points[-2], dtype=np.int32)
            pt2_int = np.array(map_points[-1], dtype=np.int32)
            cv2.line(map_img, pt1_int, pt2_int, LINE_COLOR, 2)
            cv2.circle(map_img, pt2_int, 3, LINE_COLOR, 2)
            
            # publish map image
            map_msg = CompressedImage()
            map_msg.header.stamp = node.get_clock().now().to_msg()
            map_msg.format = "jpeg"
            map_msg.data = np.array(cv2.imencode('.jpg', map_img)[1]).tostring()
            map_pub.publish(map_msg)

        pts_count += 1

try:
    main()
finally:
    np_map_points = np.array(map_points, dtype=np.float32)
    np.save(output_path, np_map_points)
    cv2.imwrite(output_path.replace(".npy", ".png"), map_image)

    node.destroy_node()
    rclpy.shutdown()
