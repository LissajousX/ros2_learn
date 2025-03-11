#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    分组启动文件，演示如何使用包含（include）功能来组合多个启动文件
    """
    # 获取包的共享目录路径
    pkg_share = get_package_share_directory('my_robot_pkg')
    
    # 包含基本启动文件，并将其放在robot1命名空间下
    robot1_launch = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'basic_launch.py')
            )
        )
    ])
    
    # 包含基本启动文件，并将其放在robot2命名空间下
    robot2_launch = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'basic_launch.py')
            )
        )
    ])
    
    # 包含高级启动文件，并传递参数
    advanced_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'advanced_launch.py')
        ),
        launch_arguments={
            'namespace': 'robot3',
            'enable_publisher': 'true',
            'enable_subscriber': 'true'
        }.items()
    )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        robot1_launch,
        robot2_launch,
        advanced_launch
    ])
