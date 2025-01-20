import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取 octomap_server 包的路径
    octomap_server_dir = get_package_share_directory('octomap_server')

    # 定义 launch 参数
    resolution = LaunchConfiguration('resolution', default='0.1')
    frame_id = LaunchConfiguration('frame_id', default='map')
    max_range = LaunchConfiguration('max_range', default='100.0')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='pointcloud_topic')

    # 创建 octomap_server 节点
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {'resolution': resolution},
            {'frame_id': frame_id},
            {'sensor_model/max_range': max_range},
            {'latch': True},
            {'pointcloud_max_z': 1000},
            {'pointcloud_min_z': 0}
        ],
        remappings=[
            ('cloud_in', pointcloud_topic)
        ]
    )

    # 返回 launch 描述
    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='0.1', description='Resolution of the octomap'),
        DeclareLaunchArgument('frame_id', default_value='map', description='Frame ID of the octomap'),
        DeclareLaunchArgument('max_range', default_value='100.0', description='Maximum range of the sensor model'),
        DeclareLaunchArgument('pointcloud_topic', default_value='pointcloud_topic', description='Pointcloud topic to subscribe to'),
        octomap_server_node
    ])