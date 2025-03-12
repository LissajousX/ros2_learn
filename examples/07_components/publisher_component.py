import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherComponent(Node):
    def __init__(self, node_name='publisher_component'):
        super().__init__(node_name)
        
        # 声明参数
        self.declare_parameter('topic_name', 'component_messages')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        # 获取参数
        topic_name = self.get_parameter('topic_name').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # 创建发布者
        self.publisher = self.create_publisher(String, topic_name, 10)
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.count = 0
        
        self.get_logger().info(f'发布者组件已初始化，发布到话题: {topic_name}')
    
    def timer_callback(self):
        # 创建消息
        msg = String()
        msg.data = f'组件消息 #{self.count}'
        
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')
        
        self.count += 1

# 这个函数是组件的入口点，用于独立运行时
def main(args=None):
    rclpy.init(args=args)
    node = PublisherComponent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# 这是组件接口，用于注册组件到ROS2组件系统
def create_publisher_component(**kwargs):
    return PublisherComponent(**kwargs)
