import rclpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2 
import numpy as np
import yaml

image_buffer = []

class PID:
    def __init__(self, kp, kd, ki):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.last_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        return self.kp*error + self.kd*derivative + self.ki*self.integral

def image_callback(msg):
    global image_buffer
    logger = node.get_logger()
    logger.info(f"Received image msg")
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_buffer.append(image_np)
    if len(image_buffer) > 10:
        logger.info(f"Buffer full, dropping oldest image")
        image_buffer = image_buffer[-10:]

def main():
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["2d_pid"]["node"]
    topic = config["2d_pid"]["topic"]
    show_topic = config["2d_pid"]["show_topic"]
    sample_rate = config["2d_pid"]["sample_rate"]
    image_topic = config["sensors"]["camera"]["topic"]
    lower_color = config["2d_pid"]["lower_color"]
    upper_color = config["2d_pid"]["upper_color"]
    area_threshold = config["2d_pid"]["area_threshold"]
    kp = config["2d_pid"]["pid"]["kp"]
    kd = config["2d_pid"]["pid"]["kd"]
    ki = config["2d_pid"]["pid"]["ki"]

    # init rosnode
    rclpy.init()
    global node 
    node = rclpy.create_node(node_name)
    image_listener = node.create_subscription(CompressedImage, image_topic, image_callback, 10)
    twist_publisher = node.create_publisher(Twist, topic, 10)
    image_publisher = node.create_publisher(CompressedImage, show_topic, 10)
    rate = node.create_rate(sample_rate)  # frequency in Hz
    logger = node.get_logger()
    rate, image_listener, twist_publisher
    logger.info('Cone detector node launched.')

    pid = PID(kp, kd, ki)

    while True:

        if len(image_buffer) > 0:
            image_np = image_buffer.pop()
            logger.info(f"Processing image")
            
        mask = cv2.inRange(image_np, lower_color, upper_color)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            
            largest_contour = contours[0]
            bb = cv2.boundingRect(largest_contour)
            largest_area = bb[2] * bb[3]
            for contour in contours:
                bb = cv2.boundingRect(contour)
                area = bb[2] * bb[3]
                if area > largest_area:
                    largest_contour = contour
                    largest_area = area

            bb = cv2.boundingRect(largest_contour)
            if largest_area > area_threshold:

                x, y, w, h = bb
                cx, cy = x + w//2, y + h//2
                height, width, _ = image_np.shape

                # draw bounding box
                bb_image = image_np.copy()
                cv2.rectangle(bb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)      
                cv2.circle(bb_image, (cx, cy), 5, (0, 0, 255), -1)
                show_img = np.hstack((image_np, bb_image))

                show_msg = CompressedImage()
                show_msg.format = "jpeg"
                show_msg.data = np.array(cv2.imencode('.jpg', show_img)[1]).tostring()
                image_publisher.publish(show_msg)

                # calculate error
                img_cx = width//2
                error = cx - img_cx

                # calculate pid
                correction = pid.update(error)
                angle = 0.0 - correction
                logger.info(f"Error: {error}, Correction: {correction}, Angle: {angle}")

                # convert to angular velocity in z
                twist = Twist()
                twist.linear.x = 0.2
                twist.angular.z = angle
                twist_publisher.publish(twist)      

if __name__ == "__main__":
    main()