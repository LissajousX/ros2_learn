#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleNode(Node):
    def __init__(self, node_name='simple_node', topic_name='simple_topic', publish_rate=1.0):
        """
        简单节点，可以作为发布者或订阅者
        """
        super().__init__(node_name)
        
        # 声明参数
        self.declare_parameter('is_publisher', True)
        self.declare_parameter('topic_name', topic_name)
        self.declare_parameter('publish_rate', publish_rate)
        
        # 获取参数值
        self.is_publisher = self.get_parameter('is_publisher').value
        self.topic_name = self.get_parameter('topic_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 根据参数配置为发布者或订阅者
        if self.is_publisher:
            self.publisher = self.create_publisher(String, self.topic_name, 10)
            self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
            self.count = 0
            self.get_logger().info(f'创建发布者节点，发布到话题: {self.topic_name}')
        else:
            self.subscription = self.create_subscription(
                String,
                self.topic_name,
                self.listener_callback,
                10)
            self.get_logger().info(f'创建订阅者节点，订阅话题: {self.topic_name}')
    
    def timer_callback(self):
        """
        定时器回调函数，用于发布消息
        """
        msg = String()
        msg.data = f'简单消息 #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')
        self.count += 1
    
    def listener_callback(self, msg):
        """
        订阅者回调函数，用于接收消息
        """
        self.get_logger().info(f'收到: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
