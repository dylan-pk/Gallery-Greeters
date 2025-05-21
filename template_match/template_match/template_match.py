#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from example_interfaces.srv import SetBool
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2 as cv
import numpy as np
import os
import sys
import argparse

# Shared function: Load templates from folder
def load_templates(directory, orb, logger=None):
    templates = {}
    for file in os.listdir(directory):
        if file.lower().endswith(('.jpg', 'png')):
            path = os.path.join(directory, file)
            image = cv.imread(path, cv.IMREAD_COLOR)
            label = os.path.splitext(file)[0].lower()

            if logger:
                logger.info(f"📂 Loading from: {path} → Label: '{label}'")
            
            if image is not None:
                kp, des = orb.detectAndCompute(image, None)
                templates[label] = {
                    'image': image,
                    'keypoints': kp,
                    'descriptors': des
                }
                if logger:
                    logger
            else:
                if logger:
                    logger.warn(f"⚠️ Failed to load template: {file}")
    return templates

# Shared function: match templates against frame
def match_templates(frame, templates, orb, logger=None, confidence_threshold=0.65):
    kp2, des2 = orb.detectAndCompute(frame, None)
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

    best_label = None
    best_confidence = 0.0
    best_matches = []

    D_max = 75  # distance normalization factor
    top_n = 20  # number of top matches to consider

    for label, data in templates.items():
        des1 = data['descriptors']
        if des1 is None or des2 is None:
            continue

        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        if len(matches) < top_n:
            continue

        avg_dist = np.mean([m.distance for m in matches[:top_n]])
        confidence = max(0.0, 1.0 - avg_dist / D_max)
        if logger:
            logger.info(f"🖼 Template '{label}' → Confidence: {confidence:.2f}")
        else:
            print(f"🖼 Template '{label}' → Confidence: {confidence:.2f}")

        # Save debug match image
        img_matches = cv.drawMatches(data['image'], data['keypoints'], frame, kp2, matches[:top_n], None, flags=2)
        cv.imwrite(f"/tmp/match_{label}.jpg", img_matches)

        if confidence > best_confidence:
            best_confidence = confidence
            best_label = label
            best_matches = matches[:top_n]

    if best_confidence > confidence_threshold:
        return best_label, best_confidence, best_matches
    else:
        return None, 0.0, []
    
class TemplateMatchingNode(Node):
    def __init__(self):
        super().__init__('template_matching_node')
        self.bridge = CvBridge()

        self.declare_parameter('confidence_threshold', 0.70)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        self.orb = cv.ORB_create(nfeatures=1000)

        resource_path = os.path.join(get_package_share_directory('template_match'), 'resource')
        self.templates = load_templates(resource_path, self.orb, logger=self.get_logger())

        self.latest_image_msg = None

        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').get_parameter_value().string_value
        self.get_logger().info(f"📸 Subscribing to image topic: {image_topic}")
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_service(SetBool, 'perform_template_matching', self.handle_template_matching)
        self.interrupt_pub = self.create_publisher(Bool, '/interrupt_signal', 10)

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

        label, confidence, _ = match_templates(frame, self.templates, self.orb, self.get_logger(), self.confidence_threshold)

        if label:
            response.success = True
            response.message = f"Matched: {label} (Confidence: {confidence:.2f})"

            interrupt_msg = Bool()
            interrupt_msg.data = True
            self.interrupt_pub.publish(interrupt_msg)
            self.get_logger().warn(f"📢 Interrupt signal published due to match: {label}")
        else:
            response.success = False
            response.message = "No matching object found."

        return response


def run_webcam_mode():
    orb = cv.ORB_create(nfeatures=1000)
    resource_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resource'))
    templates = load_templates(resource_dir, orb)

    cap = cv.VideoCapture(0)
    print("📷 Webcam test mode — Press 'q' to exit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to read from webcam")
            break

        label, confidence, _ = match_templates(frame, templates, orb)
        if label:
            cv.putText(frame, f"{label} ({confidence:.2f})", (10, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv.imshow("Webcam Match", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='Template Matching Node or Webcam Tester')
    parser.add_argument('--test-camera', action='store_true', help='Run with webcam instead of ROS image topic')
    args, ros_args = parser.parse_known_args()

    if args.test_camera:
        run_webcam_mode()
    else:
        rclpy.init(args=ros_args)
        node = TemplateMatchingNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()