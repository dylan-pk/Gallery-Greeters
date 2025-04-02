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
        
        # Get package directory and resource paths  
        # package_path = get_package_share_directory('template_match')
        package_path = os.path.join(os.path.dirname(__file__), 'resource')

        # image file must be named 'wall.jpg'
        image_path = os.path.join(package_path, 'wall.jpg')      
        # template image must be called 'template.jpg'
        template_path = os.path.join(package_path, 'template.jpg')      

        # Load images in color
        img = cv.imread(image_path, cv.IMREAD_COLOR)
        template = cv.imread(template_path, cv.IMREAD_COLOR)
            
        if img is None:
            self.get_logger().error(f"Could not read {image_path}. Check the file path!")
            return
        if template is None:
            self.get_logger().error(f"Could not read {template_path}. Check the file path!")
            return

        # Get width and height of template image
        w, h, _ = template.shape  

        # Split image and template RGB channels
        img_b, img_g, img_r = cv.split(img)
        temp_b, temp_g, temp_r = cv.split(template)

        # All possible methods for template matching
        methods = ['TM_CCOEFF', 'TM_CCOEFF_NORMED', 'TM_CCORR', 'TM_CCORR_NORMED', 'TM_SQDIFF', 'TM_SQDIFF_NORMED']

        for meth in methods:
            img_copy = img.copy()
            method = getattr(cv, meth)

            # Apply template matching on each channel
            res_b = cv.matchTemplate(img_b, temp_b, method)
            res_g = cv.matchTemplate(img_g, temp_g, method)
            res_r = cv.matchTemplate(img_r, temp_r, method)

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
            matched_img = img.copy()
            cv.rectangle(matched_img, top_left, bottom_right, (0, 255, 0), 2)

            # Convert the OpenCV image to a ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(matched_img, encoding="bgr8")
            self.image_publisher.publish(ros_image)

            # save result to disk
            result_image_path = os.path.join(package_path, f"matched_{meth}.jpg")
            cv.imwrite(result_image_path, matched_img)

            # Logging the progress
            self.get_logger().info(f"Color template matching completed using {meth}. Image published to 'matched_image' topic.")
            self.get_logger().info(f"Result saved to {result_image_path}")

def main(args=None):
    rclpy.init(args=args)
    node = TemplateMatchingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
