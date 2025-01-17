import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'offboard_pkg'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    # 定义参数
    world_file = os.path.expanduser(os.path.join(pkg_share, 'worlds/myworld.world'))

    # 启动Gazebo并加载world文件
    gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )

    # 启动RViz2
    rviz_cmd = ExecuteProcess(
        cmd=['rviz2'],
        output='screen'
    )

    # 启动robot_state_publisher节点
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 定义launch描述
    ld = LaunchDescription([
        DeclareLaunchArgument('world_file', default_value=world_file),
        gazebo_cmd,
        rviz_cmd,
        robot_state_publisher_cmd
    ])

    return ld