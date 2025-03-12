#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class QoSSubscriber(Node):
    """
    演示不同QoS设置的订阅者
    """
    def __init__(self):
        super().__init__('qos_subscriber')
        
        # 创建不同QoS配置的订阅者
        
        # 1. 默认QoS配置
        self.default_subscription = self.create_subscription(
            String,
            'default_topic',
            self.default_callback,
            10)
        
        # 2. 可靠传输QoS配置
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.reliable_subscription = self.create_subscription(
            String,
            'reliable_topic',
            self.reliable_callback,
            reliable_qos)
        
        # 3. 尽力传输QoS配置（适合传感器数据）
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.best_effort_subscription = self.create_subscription(
            String,
            'best_effort_topic',
            self.best_effort_callback,
            best_effort_qos)
        
        # 4. 持久化QoS配置（适合配置和参数）
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.transient_subscription = self.create_subscription(
            String,
            'transient_topic',
            self.transient_callback,
            transient_qos)
        
        # 使用预定义的QoS配置
        from rclpy.qos import qos_profile_sensor_data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_topic',
            self.sensor_callback,
            qos_profile_sensor_data)
        
        self.get_logger().info('QoS订阅者已启动')
    
    def default_callback(self, msg):
        """
        默认QoS主题的回调函数
        """
        self.get_logger().info(f'收到默认QoS消息: {msg.data}')
    
    def reliable_callback(self, msg):
        """
        可靠QoS主题的回调函数
        """
        self.get_logger().info(f'收到可靠QoS消息: {msg.data}')
    
    def best_effort_callback(self, msg):
        """
        尽力QoS主题的回调函数
        """
        self.get_logger().info(f'收到尽力QoS消息: {msg.data}')
    
    def transient_callback(self, msg):
        """
        持久化QoS主题的回调函数
        """
        self.get_logger().info(f'收到持久化QoS消息: {msg.data}')
    
    def sensor_callback(self, msg):
        """
        传感器QoS主题的回调函数
        """
        self.get_logger().info(f'收到传感器QoS消息: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    qos_subscriber = QoSSubscriber()
    
    try:
        rclpy.spin(qos_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        qos_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
