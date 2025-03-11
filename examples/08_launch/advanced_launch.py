#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    高级启动文件，演示命名空间、参数文件和条件启动等功能
    """
    # 获取包的共享目录路径
    pkg_share = get_package_share_directory('my_robot_pkg')
    
    # 声明启动参数
    enable_publisher_arg = DeclareLaunchArgument(
        'enable_publisher',
        default_value='true',
        description='是否启动发布者节点'
    )
    
    enable_subscriber_arg = DeclareLaunchArgument(
        'enable_subscriber',
        default_value='true',
        description='是否启动订阅者节点'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='节点的命名空间'
    )
    
    # 获取启动参数的值
    enable_publisher = LaunchConfiguration('enable_publisher')
    enable_subscriber = LaunchConfiguration('enable_subscriber')
    namespace = LaunchConfiguration('namespace')
    
    # 创建发布者节点，使用条件启动
    publisher_node = Node(
        package='my_robot_pkg',
        executable='simple_node',
        name='publisher_node',
        namespace=namespace,
        parameters=[{
            'is_publisher': True,
            'topic_name': 'advanced_topic',
            'publish_rate': 2.0
        }],
        output='screen',
        condition=IfCondition(enable_publisher)
    )
    
    # 创建订阅者节点，使用条件启动
    subscriber_node = Node(
        package='my_robot_pkg',
        executable='simple_node',
        name='subscriber_node',
        namespace=namespace,
        parameters=[{
            'is_publisher': False,
            'topic_name': 'advanced_topic'
        }],
        output='screen',
        condition=IfCondition(enable_subscriber)
    )
    
    # 创建参数节点，从参数文件加载参数
    param_node = Node(
        package='my_robot_pkg',
        executable='param_node',
        name='param_node',
        namespace=namespace,
        parameters=[os.path.join(pkg_share, 'config', 'params.yaml')],
        output='screen'
    )
    
    # 创建一个执行进程，在启动文件中执行命令
    echo_process = ExecuteProcess(
        cmd=['echo', '启动文件已成功启动所有节点！'],
        output='screen'
    )
    
    # 创建事件处理器，在echo_process退出后执行
    echo_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=echo_process,
            on_exit=[ExecuteProcess(
                cmd=['echo', '所有节点都已启动，可以开始使用了！'],
                output='screen'
            )]
        )
    )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        enable_publisher_arg,
        enable_subscriber_arg,
        namespace_arg,
        publisher_node,
        subscriber_node,
        param_node,
        echo_process,
        echo_exit_event_handler
    ])
