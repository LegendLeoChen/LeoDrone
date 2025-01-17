import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            '/chase_cam/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 显示图像
        cv2.imshow('Image Viewer', cv_image)
        cv2.waitKey(1)  # 等待1毫秒，以便图像可以显示

def main(args=None):
    rclpy.init(args=args)
    image_viewer_node = ImageViewerNode()
    rclpy.spin(image_viewer_node)
    image_viewer_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()