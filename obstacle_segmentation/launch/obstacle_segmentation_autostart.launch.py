# 导入库
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    param_yaml_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(get_package_share_directory('robot_bring_up'), 'config', 'robot.yaml')
    )
    declare_yaml_path = DeclareLaunchArgument(
        'params_file',
        default_value=param_yaml_path,
        description='Full path to the configuration file to load'
    )
    node_01 = Node(
        package="obstacle_segmentation",
        executable="obstacle_segmentation_node",
        output="screen",
        namespace="",
        parameters=[param_yaml_path],
        name="obstacle_segmentation_node"
    )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [declare_yaml_path, node_01]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description