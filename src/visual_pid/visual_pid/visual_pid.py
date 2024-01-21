import rclpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2 
import numpy as np
import yaml
import torch 

image_buffer = []

def image_callback(msg):
    global image_buffer
    logger = node.get_logger()
    logger.info(f"Received image msg")
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_buffer.append(image_np)
    if len(image_buffer) > 10:
        logger.info(f"Buffer full, dropping oldest image")
        image_buffer = image_buffer[-10:]

def main():
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["visual_pid"]["node"]
    topic = config["visual_pid"]["topic"]
    show_topic = config["visual_pid"]["show_topic"]
    sample_rate = config["visual_pid"]["sample_rate"]
    image_topic = config["sensors"]["camera"]["topic"]
    correction = config["visual_pid"]["correction"]
    conf_threshold = config["visual_pid"]["confidence_threshold"]
    error_threshold = config["visual_pid"]["error_threshold"]

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

    # load model 
    obj_detector = torch.hub.load('ultralytics/yolov5', 'custom', '/home/user/ws/src/models/yolov5_finetuned/best.pt')
    logger.info('Model loaded.')

    while True:

        if len(image_buffer) > 0:
            camera_image = image_buffer.pop()
            out_image = camera_image.copy()
            logger.info(f"Processing image")
            
            # detect cone
            inference_results = obj_detector(camera_image)

            if len(inference_results) > 0:
                
                # get bounding box and center
                box = inference_results.xyxy[0].numpy()
                x1, y1, x2, y2, conf, cls = box
                cx, cy = (x1+x2)//2, (y1+y2)//2
                height, width, _ = camera_image.shape

                # draw bounding box
                cv2.rectangle(out_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)   
                cv2.circle(out_image, (cx, cy), 5, (0, 0, 255), -1)

                if conf < conf_threshold:
                    logger.info(f"Confidence below threshold, skipping correction")
                    continue

                # calculate error and correction
                img_cx = width//2
                error = cx - img_cx
                angular_correction = correction if np.abs(error) > error_threshold else 0
                angular_correction *= np.sign(error)

                logger.info(f"Error: {error}, Correction: {angular_correction}")

                # publish twist message
                twist = Twist()
                twist.linear.x = 0.2
                twist.angular.z = angular_correction
                twist_publisher.publish(twist)   

            # publish image to topic
            show_msg = CompressedImage()
            show_msg.format = "jpeg"
            show_msg.data = np.array(cv2.imencode('.jpg', out_image)[1]).tostring()
            image_publisher.publish(show_msg)

        rclpy.spin_once(node)   

if __name__ == "__main__":
    main()