#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from cv_bridge import CvBridge

import cv2

import torch
from obj_detector.pytorch_yolov4 import models as yolov4
from obj_detector.pytorch_yolov4.tool.utils import do_detect

class ConeDetector(Node):
    
        def __init__(self):
            super().__init__('cone_detector')
            self.logger = self.get_logger()
            self.logger.info('Initializing cone detector node...')
    
            # Load config
            fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
            image_config = fs.getNode("sensors").getNode("camera")
            self.img_topic = image_config.getNode("topic").string()
            self.cam_fps = int(image_config.getNode("sample_rate").real())
            self.img_size = (int(image_config.getNode("resolution").getNode("width").real()), int(image_config.getNode("resolution").getNode("height").real()))
            
            detector_config = fs.getNode("cone_detector")
            self.detection_topic = detector_config.getNode("topic").string()
            self.sample_rate = int(detector_config.getNode("sample_rate").real())
            self.confidence_threshold = detector_config.getNode("confidence_threshold").real()
            self.error_threshold = int(detector_config.getNode("error_threshold").real())
            model_path = detector_config.getNode("model_path").string()
            self.debug_img = bool(detector_config.getNode("debug_img").real())
            self.debug_topic = detector_config.getNode("debug_topic").string()
            fs.release()

            self.logger.info("Initializing YoloV4...")    
            # Check CUDA avaliability
            if torch.cuda.is_available():
                self.logger.info("CUDA is available.")
                self.logger.info(f"Device: {torch.cuda.get_device_name(0)}")
            else:
                self.logger.warn("CUDA is not available. Will use CPU.")

            self.model = yolov4.Yolov4(n_classes=1)
            device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            pretrained_dict = torch.load(model_path, map_location=device)
            self.model.load_state_dict(pretrained_dict)
            self.model.cuda()

            self.logger.info("Model loaded successfully.")
    
            # Init publishers
            self.image_listener = self.create_subscription(Image, self.img_topic, self.image_callback, 10)
            self.detection_publisher = self.create_publisher(Detection2DArray, self.detection_topic, 10)
            self.debug_publisher = self.create_publisher(Image, self.debug_topic, 10)
            self.timer = self.create_timer(1/self.sample_rate, self.timer_callback)

            self.current_img_msg = None
            self.current_image = None
            self.bridge = CvBridge()
    
            self.logger.info('Cone detector node launched.')
    
        def timer_callback(self):
            # Detect cone
            if self.current_image is not None:
                self.logger.info("Running inference...")
                inference_results = do_detect(self.model, self.current_image, 0.5, 1, 0.4, True)
    
                # Get bounding box and center
                if len(inference_results) > 0:
                    self.logger.info("Found cone in image.")
                    box = inference_results.xyxy[0].numpy()
                    x1, y1, x2, y2, conf, obj_cls = box

                    if conf < self.confidence_threshold:
                        self.logger.warn(f"Confidence {conf} below threshold {self.confidence_threshold}, skipping...")
                        return

                    cx, cy = (x1+x2)//2, (y1+y2)//2
                    height, width, _ = self.current_image.shape

                    # Calculate error and correction
                    img_cx = width//2
                    error = cx - img_cx
                    self.logger.info(f"Center point error: {error}")

                    if self.debug_img:
                        # Draw bounding box
                        cv2.rectangle(self.current_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)   
                        cv2.circle(self.current_image, (cx, cy), 5, (0, 0, 255), -1)
                        # Draw center point
                        cv2.circle(self.current_image, (width//2, height//2), 5, (255, 0, 0), -1)
                        cv2.circle(self.current_image, (cx, cy), 5, (0, 0, 255), -1)
                        # Draw error line
                        cv2.line(self.current_image, (width//2, height//2), (cx, cy), (255, 0, 0), 2)
                        cv2.text(self.current_image, f"Error: {error}", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                        debug_img_msg = self.bridge.cv2_to_imgmsg(self.current_image, encoding="bgr8")
                        self.debug_publisher.publish(debug_img_msg)
    
                    # Publish detection
                    detection = Detection2D()
                    detection.header.stamp = self.get_clock().now().to_msg()
                    detection.bbox.center.x = cx
                    detection.bbox.center.y = cy
                    detection.bbox.size_x = x2 - x1
                    detection.bbox.size_y = y2 - y1
                    detection.results.id = obj_cls
                    detection.results.score = conf
                    detection.results.pose.position.x = error

                    self.detection_publisher.publish(detection)
                else:
                    self.logger.info("No cone detected.")

        def image_callback(self, msg):
            # Convert ROS Image to OpenCV Image
            self.logger.info("Receiving camera images...", once=True)
            self.current_image_msg = msg
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

if __name__ == "__main__":
    rclpy.init(args=None)

    cone_detector = ConeDetector()

    rclpy.spin(cone_detector)
    cone_detector.destroy_node()
    rclpy.shutdown()

