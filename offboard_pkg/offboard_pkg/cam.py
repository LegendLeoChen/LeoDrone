import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import requests
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_image)  # 30 FPS
        self.url = "http://手机IP地址/mjpegfeed"  # 确保 URL 正确
        self.bridge = CvBridge()

    def publish_image(self):
        try:
            # 发送 HTTP 请求获取图像
            response = requests.get(self.url, stream=True)
            response.raise_for_status()

            # 逐帧读取 MJPEG 流
            byte_stream = b''
            for chunk in response.iter_content(1024):
                byte_stream += chunk
                a = byte_stream.find(b'\xff\xd8')
                b = byte_stream.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = byte_stream[a:b+2]
                    byte_stream = byte_stream[b+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        # 将 OpenCV 图像转换为 ROS 2 图像消息
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        self.publisher.publish(msg)
                        self.get_logger().info('Published image')

                        # 显示图像
                        cv2.imshow('DroidCam', frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.get_logger().info('User requested to quit')
                            rclpy.shutdown()
                    else:
                        self.get_logger().error('Failed to decode image')
        except requests.RequestException as e:
            self.get_logger().error(f'Failed to capture image: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # 确保在节点关闭时销毁所有 OpenCV 窗口

if __name__ == '__main__':
    main()