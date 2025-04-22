#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TemplateMatchingNode(Node):
    def __init__(self):
        super().__init__('template_matching_node')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'matched_image', 10)
        
        # Subscribe to Gazebo camera feed   # message type, topic, callback func, 10 QoS queue size
        self.create_subscription(Image, '/camera/image_raw', self.iamge_callback, 10)
        
        # Publisher for matched image
        self.image_publisher = self.create_publisher(Image, 'matched_image', 10)

        # Get package directory and resource paths  
        # Load template image once
        package_path = os.path.join(os.path.dirname(__file__), 'resource')
        # template image must be called 'template.jpg'
        template_path = os.path.join(package_path, 'template.jpg') 
        # read image in colour     
        self.template = cv.imread(template_path, cv.IMREAD_COLOR)

            

        if self.template is None:
            self.get_logger().error(f"Could not read {template_path}. Check the file path!")
            raise FileNotFoundError("!Template image not found!")
        
        self.template_h, self.template_w = self.template.shape
        self.temp_b, self.temp_g, self.temp_r = cv.split(self.template)

def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Split image channels
        img_b, img_g, img_r = cv.split(frame)

        # All possible methods for template matching
        methods = ['TM_CCOEFF', 'TM_CCOEFF_NORMED', 'TM_CCORR', 'TM_CCORR_NORMED', 'TM_SQDIFF', 'TM_SQDIFF_NORMED']

        for meth in methods:
            method = getattr(cv, meth)

            # Apply template matching on each channel
            res_b = cv.matchTemplate(img_b, self.temp_b, method)
            res_g = cv.matchTemplate(img_g, self.temp_g, method)
            res_r = cv.matchTemplate(img_r, self.temp_r, method)

            # Average the results from all channels
            res = (res_b + res_g + res_r) / 3

            # Get the best match location
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)

            if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
                top_left = min_loc  # For these methods, lower values are better
            else:
                top_left = max_loc  # For other methods, higher values are better

            bottom_right = (top_left[0] + h, top_left[1] + w)

            # Draw rectangle on the detected match
            matched_img = frame.copy()
            cv.rectangle(matched_img, top_left, bottom_right, (0, 255, 0), 2)

            # Convert to ROS Image and publish
            ros_image = self.bridge.cv2_to_imgmsg(matched_img, encoding="bgr8")
            self.image_publisher.publish(ros_image)

            # Convert the OpenCV image to a ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(matched_img, encoding="bgr8")
            self.image_publisher.publish(ros_image)

            # Logging the progress
            self.get_logger().info(f"Color template matching completed using {meth}. Image published to 'matched_image' topic.")

def main(args=None):
    rclpy.init(args=args)
    node = TemplateMatchingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
