import rclpy
import yaml
import numpy as np
import cv2
from libs.mono_video_odometry import MonoVideoOdometery
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped

image_buffer = []

def camera_callback(msg):
    global node
    global image_buffer
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_buffer.append(image_np)

def main():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["vo"]["node"]
    topic = config["vo"]["topic"]
    sample_rate = config["vo"]["sample_rate"]
    camera_topic = config["sensors"]["camera"]["topic"]
    camera_fps = config["sensors"]["camera"]["fps"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node(node_name)
    vo_pub = node.create_publisher(PoseStamped, topic, 10)
    camera_sub = node.create_subscription(CompressedImage, camera_topic, camera_callback, 10)
    rate = node.create_rate(sample_rate) # frequency in Hz
    camera_sub, vo_pub, rate
    logger = node.get_logger()
    logger.info('Visual odometry node launched.')


    # get first frame
    global image_buffer
    while len(image_buffer) == 0:
        rclpy.spin_once(node)

    first_frame = image_buffer.pop(0)
    gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
    vo = MonoVideoOdometery(gray)

    while True:
        rclpy.spin_once(node)

        if len(image_buffer) > 0:
            curr_frame = image_buffer.pop(0)
            
            gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
            T = vo.visual_odometery(gray)

            if T is not None:
                # convert homogenous coordinates to pose
                position = T[:3, 3]
                
                R = T[:3, :3]
                w = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
                x = (R[2,1] - R[1,2]) / (4*w)
                y = (R[0,2] - R[2,0]) / (4*w)
                z = (R[1,0] - R[0,1]) / (4*w)
                
                # publish pose message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = node.get_clock().now().to_msg()

                pose_msg.pose.position.x = position[0]
                pose_msg.pose.position.y = position[1]
                pose_msg.pose.position.z = position[2]

                pose_msg.pose.orientation.x = x
                pose_msg.pose.orientation.y = y
                pose_msg.pose.orientation.z = z
                pose_msg.pose.orientation.w = w

                vo_pub.publish(pose_msg)
            else:
                logger.info("Visual odometry failed. No pose calculated.")
        

if __name__ == "__main__":
    main()