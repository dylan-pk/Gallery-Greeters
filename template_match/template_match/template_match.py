#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.callback_groups import ReentrantCallbackGroup
import cv2 as cv
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.srv import SetBool  # Service type: You can change this to any custom service type.

class TemplateMatchingNode(Node):
    def __init__(self):
        super().__init__('template_matching_node')
        self.bridge = CvBridge()
        
        # Service to start template matching
        self.srv = self.create_service(SetBool, 'perform_template_matching', self.handle_template_matching)

        # Load template image
        package_path = os.path.join(os.path.dirname(__file__), 'resource')
        template_path = os.path.join(package_path, 'template.jpg')
        self.template = cv.imread(template_path, cv.IMREAD_COLOR)

        if self.template is None:
            self.get_logger().error(f"Could not read {template_path}. Check the file path!")
            raise FileNotFoundError("Template image not found.")

        self.template_h, self.template_w = self.template.shape[:2]
        self.temp_b, self.temp_g, self.temp_r = cv.split(self.template)

        # Subscribe to camera feed
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # A place to store the latest image for template matching
        self.latest_image = None

    def image_callback(self, msg):
        """Callback function that updates the latest camera image."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = frame

    def handle_template_matching(self, request, response):
        """Handle the service call to perform template matching."""
        if self.latest_image is None:
            response.success = False
            response.message = "No image available for matching."
            return response

        # Perform template matching
        result_message = self.match_template(self.latest_image)

        # Update response based on result
        if result_message:
            response.success = True
            response.message = f"Object matched: {result_message}"
        else:
            response.success = False
            response.message = "No match found."

        return response

    def match_template(self, frame):
        """Perform template matching and return the name of the matched object."""
        img_b, img_g, img_r = cv.split(frame)
        methods = ['TM_CCOEFF_NORMED']  # You can try other methods if needed

        for meth in methods:
            method = getattr(cv, meth)
            res_b = cv.matchTemplate(img_b, self.temp_b, method)
            res_g = cv.matchTemplate(img_g, self.temp_g, method)
            res_r = cv.matchTemplate(img_r, self.temp_r, method)

            res = (res_b + res_g + res_r) / 3.0
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)

            if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
                top_left = min_loc
            else:
                top_left = max_loc

            bottom_right = (top_left[0] + self.template_w, top_left[1] + self.template_h)

            matched_img = frame.copy()
            cv.rectangle(matched_img, top_left, bottom_right, (0, 255, 0), 2)

            # Check if match is above a certain threshold, and if so, we have a valid match
            threshold = 0.8
            if max_val > threshold:
                self.get_logger().info(f"Template matched with value: {max_val}")
                return "Duchess"  # You can return a name or label corresponding to the object

        # No match found
        return None


def main(args=None):
    rclpy.init(args=args)
    node = TemplateMatchingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
