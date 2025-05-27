import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class SmartCamNode(Node):
    def __init__(self):
        super().__init__('smart_cam_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = self.find_working_camera()
        if not self.cap or not self.cap.isOpened():
            return

        self.get_logger().info("✅ Camera opened successfully.")
        self.timer = self.create_timer(1/15, self.timer_callback)

    def find_working_camera(self):
        self.get_logger().info("🔍 Scanning for available video devices...")
        for index in range(5):  # Try /dev/video0 to /dev/video4
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 15)

                ret, _ = cap.read()
                if ret:
                    self.get_logger().info(f"📷 Using /dev/video{index}")
                    return cap
                cap.release()
        return None

    def timer_callback(self):
        if not self.cap:
            return
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("⚠️ Frame grab failed.")

def main(args=None):
    rclpy.init(args=args)
    node = SmartCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
