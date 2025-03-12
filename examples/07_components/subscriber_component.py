import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberComponent(Node):
    def __init__(self, node_name='subscriber_component'):
        super().__init__(node_name)
        
        # 声明参数
        self.declare_parameter('topic_name', 'component_messages')
        
        # 获取参数
        topic_name = self.get_parameter('topic_name').value
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10
        )
        
        self.get_logger().info(f'订阅者组件已初始化，订阅话题: {topic_name}')
    
    def listener_callback(self, msg):
        self.get_logger().info(f'收到: {msg.data}')

# 这个函数是组件的入口点，用于独立运行时
def main(args=None):
    rclpy.init(args=args)
    node = SubscriberComponent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# 这是组件接口，用于注册组件到ROS2组件系统
def create_subscriber_component(**kwargs):
    return SubscriberComponent(**kwargs)
