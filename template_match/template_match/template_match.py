#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from example_interfaces.srv import SetBool
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2 as cv
import numpy as np
import os


class TemplateMatchingNode(Node):
    def __init__(self):
        super().__init__('template_matching_node')
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # Thresholds
        self.declare_parameter('confidence_threshold', 0.65)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # ORB detector
        self.orb = cv.ORB_create(nfeatures=1000)

        # Load templates
        resource_path = os.path.join(get_package_share_directory('template_match'), 'resource')
        self.templates = {}
        self.load_templates(resource_path)

        # Image message store
        self.latest_image_msg = None

        # ROS Interfaces
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_service(SetBool, 'perform_template_matching', self.handle_template_matching)

    def load_templates(self, directory):
        for file in os.listdir(directory):
            if file.lower().endswith(('.jpg', '.png')):
                path = os.path.join(directory, file)
                image = cv.imread(path, cv.IMREAD_COLOR)
                if image is not None:
                    kp, des = self.orb.detectAndCompute(image, None)
                    label = os.path.splitext(file)[0]
                    self.templates[label] = {
                        'image': image,
                        'keypoints': kp,
                        'descriptors': des
                    }
                    self.get_logger().info(f"✅ Loaded template '{label}'")
                else:
                    self.get_logger().warn(f"⚠️ Failed to load template: {file}")

    def image_callback(self, msg):
        self.latest_image_msg = msg
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv.imwrite('/tmp/latest_camera_frame.jpg', frame)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def handle_template_matching(self, request, response):
        if self.latest_image_msg is None:
            response.success = False
            response.message = "No image received yet."
            return response

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
        except Exception as e:
            response.success = False
            response.message = f"Failed to convert image: {e}"
            return response

        label, confidence, matches = self.match_templates(frame)

        if label:
            response.success = True
            response.message = f"Matched: {label} (Confidence: {confidence:.2f})"
        else:
            response.success = False
            response.message = "No matching object found."

        return response

    def match_templates(self, frame):
        kp2, des2 = self.orb.detectAndCompute(frame, None)
        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        best_label = None
        best_confidence = 0.0
        best_matches = []

        D_max = 75  # distance normalization factor
        top_n = 20  # number of top matches to consider

        for label, data in self.templates.items():
            des1 = data['descriptors']
            if des1 is None or des2 is None:
                continue

            matches = bf.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) < top_n:
                continue

            avg_dist = np.mean([m.distance for m in matches[:top_n]])
            confidence = max(0.0, 1.0 - avg_dist / D_max)
            self.get_logger().info(f"🖼 Template '{label}' → Confidence: {confidence:.2f}")

            # Save debug match image
            img_matches = cv.drawMatches(data['image'], data['keypoints'], frame, kp2, matches[:top_n], None, flags=2)
            cv.imwrite(f"/tmp/match_{label}.jpg", img_matches)

            if confidence > best_confidence:
                best_confidence = confidence
                best_label = label
                best_matches = matches[:top_n]

        if best_confidence > self.confidence_threshold:
            return best_label, best_confidence, best_matches
        else:
            return None, 0.0, []


def main(args=None):
    rclpy.init(args=args)
    node = TemplateMatchingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
