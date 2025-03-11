#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    基本启动文件，启动一个发布者节点和一个订阅者节点
    """
    # 创建发布者节点
    publisher_node = Node(
        package='my_robot_pkg',
        executable='simple_node',
        name='publisher_node',
        parameters=[{
            'is_publisher': True,
            'topic_name': 'launch_topic',
            'publish_rate': 2.0
        }],
        output='screen'
    )
    
    # 创建订阅者节点
    subscriber_node = Node(
        package='my_robot_pkg',
        executable='simple_node',
        name='subscriber_node',
        parameters=[{
            'is_publisher': False,
            'topic_name': 'launch_topic'
        }],
        output='screen'
    )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])
