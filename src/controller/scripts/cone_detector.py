#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

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

            self.color_lower = np.array([int(detector_config.getNode("color_lower").at(i).real()) for i in range(3)])
            self.color_upper = np.array([int(detector_config.getNode("color_upper").at(i).real()) for i in range(3)])
            
            self.debug_img = bool(detector_config.getNode("debug_image").real())
            self.debug_topic = detector_config.getNode("debug_topic").string()
            
            self.load_intrinsics(detector_config.getNode("intrinsics").string())

            fs.release()
            
            # Init publishers
            self.image_listener = self.create_subscription(Image, self.img_topic, self.image_callback, 10)
            self.detections_publisher = self.create_publisher(Detection2D, self.detection_topic, 10)
            self.debug_publisher = self.create_publisher(Image, self.debug_topic, 10)
            self.timer = self.create_timer(1/self.sample_rate, self.timer_callback)

            self.current_img_msg = None
            self.current_image = None
            self.bridge = CvBridge()
    
            self.logger.info('Cone detector node launched.')

        def load_intrinsics(self, intrinsics_path):
            fs = cv2.FileStorage(intrinsics_path, cv2.FileStorage_READ)
            self.camera_mat = fs.getNode("K").mat()
            self.dist_coeffs = fs.getNode("D").mat()
            fs.release()
    
        def timer_callback(self):
            # Detect cone
            if self.current_image is not None:
                self.logger.info("Started calculatng error...", once=True)

                mask = cv2.inRange(self.current_image, self.color_lower, self.color_upper)

                #mask = cv2.erode(mask, np.ones((5, 5)))
                mask = cv2.dilate(mask, np.ones((5, 5)))

                # debug_img_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
                # self.debug_publisher.publish(debug_img_msg)

                # find largest contour
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                largest_contour = None
                largest_size = 0
                for contour in contours:
                    c_size = cv2.contourArea(contour)
                    if c_size > largest_size:
                        largest_size = c_size
                        largest_contour = contour

                # print(largest_size)
                
                # Get bounding box and center
                if largest_contour is not None: #and cv2.contourArea(largest_contour) > 1000:
                    x1, y1, w, h = cv2.boundingRect(largest_contour)
                    x2, y2 = x1 + w, y1 + h

                    cx, cy = (x1+x2)//2, (y1+y2)//2
                    height, width, _ = self.current_image.shape

                    # Calculate error and correction
                    img_cx = width//2
                    error = cx - img_cx
                    #self.logger.info(f"Center point error: {error}")

                    if self.debug_img:
                        # Draw bounding box
                        cv2.rectangle(self.current_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)   
                        cv2.circle(self.current_image, (cx, cy), 5, (0, 0, 255), -1)
                        # Draw center point
                        cv2.circle(self.current_image, (width//2, height//2), 5, (255, 0, 0), -1)
                        cv2.circle(self.current_image, (cx, cy), 5, (0, 0, 255), -1)
                        # Draw error line
                        cv2.line(self.current_image, (width//2, height//2), (cx, cy), (255, 0, 0), 2)
                        # cv2.text(self.current_image, f"Error: {error}", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                        # self.logger.info("Publishing debug image...")
    
                    # Publish detection
                    detection = Detection2D()
                    detection.header.stamp = self.get_clock().now().to_msg()
                    detection.bbox.center.position.x = float(cx)
                    detection.bbox.center.position.y = float(cy)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    detection.results = [ObjectHypothesisWithPose()]
                    detection.results[0].pose.pose.position.x = float(error)
                    self.detections_publisher.publish(detection)

                    if self.debug_img:
                        concat = np.hstack([self.current_image, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])

                        debug_img_msg = self.bridge.cv2_to_imgmsg(concat, encoding="bgr8")
                        self.debug_publisher.publish(debug_img_msg)

            
        def image_callback(self, msg):
            # Convert ROS Image to OpenCV Image
            self.current_image_msg = msg
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Undistort image
            # self.current_image = cv2.undistort(self.current_image, self.camera_mat, self.dist_coeffs)

if __name__ == "__main__":
    rclpy.init(args=None)

    cone_detector = ConeDetector()

    rclpy.spin(cone_detector)
    cone_detector.destroy_node()
    rclpy.shutdown()
