from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    启动组件容器并加载组件
    """
    # 创建组件容器
    container = ComposableNodeContainer(
        name='component_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 添加发布者组件
            ComposableNode(
                package='my_robot_pkg',
                plugin='my_robot_pkg.publisher_component',  # 对应setup.py中注册的组件名
                name='composed_publisher',
                parameters=[{
                    'topic_name': 'composed_topic',
                    'publish_rate': 2.0
                }]
            ),
            # 添加订阅者组件
            ComposableNode(
                package='my_robot_pkg',
                plugin='my_robot_pkg.subscriber_component',  # 对应setup.py中注册的组件名
                name='composed_subscriber',
                parameters=[{
                    'topic_name': 'composed_topic'
                }]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
