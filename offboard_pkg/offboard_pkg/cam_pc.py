import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_image)  # 30 FPS
        self.cap = cv2.VideoCapture(0)  # 使用默认摄像头
        self.bridge = CvBridge()

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            # 将 OpenCV 图像转换为 ROS 2 图像消息
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info('Published image')
        else:
            self.get_logger().error('Failed to capture image')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()