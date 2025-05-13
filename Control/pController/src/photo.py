import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import glob

class ImageSaver(Node):
    def __init__(self, prefix='art'):
        super().__init__('image_saver')
        self.prefix = prefix
        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change if needed
            self.image_callback,
            10
        )

        # Subscribe to save signal topic
        self.save_signal_sub = self.create_subscription(
            Bool,
            '/save_image_signal',
            self.save_callback,
            10
        )

    def image_callback(self, msg):
        self.latest_image = msg

    def save_callback(self, msg):
        if msg.data and self.latest_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                existing_files = glob.glob(f'{self.prefix}_*.jpg')
                next_index = len(existing_files) + 1
                filename = f'{self.prefix}_{next_index}.jpg'
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Saved image as: {filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to save image: {e}')
        elif not self.latest_image:
            self.get_logger().warn('No image received yet.')

def main(args=None):
    rclpy.init(args=args)
    prefix = 'Squares'  # You can make this configurable if needed
    image_saver = ImageSaver(prefix=prefix)
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
