import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d


class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'pointcloud_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)  # 每秒发布一次

        # 设置 pcd 文件路径
        self.pcd_path = '/home/leo/leofile/uav/map.pcd'  # 替换为你的 pcd 文件路径
        self.get_logger().info(f'PCD file path set to: {self.pcd_path}')

    def publish_pointcloud(self):
        # 每次发布时重新加载 pcd 文件
        self.get_logger().info(f'Reading PCD file from: {self.pcd_path}')
        self.pcd = o3d.io.read_point_cloud(self.pcd_path)

        # 检查点云是否为空
        if len(self.pcd.points) == 0:
            self.get_logger().error('The PCD file is empty or invalid!')
            return

        points = np.asarray(self.pcd.points, dtype=np.float32)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1  # 点云是非结构化的
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 每个点占用 12 字节
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        msg.is_dense = False  # 如果点云中可能包含无效点（NaN），设置为 False

        self.publisher.publish(msg)
        self.get_logger().info(f'Published PointCloud2 message with {len(points)} points')


def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()