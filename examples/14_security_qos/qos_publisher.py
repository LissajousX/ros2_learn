#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import time

class QoSPublisher(Node):
    """
    演示不同QoS设置的发布者
    """
    def __init__(self):
        super().__init__('qos_publisher')
        
        # 创建不同QoS配置的发布者
        
        # 1. 默认QoS配置
        self.default_publisher = self.create_publisher(
            String, 'default_topic', 10)
        
        # 2. 可靠传输QoS配置
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.reliable_publisher = self.create_publisher(
            String, 'reliable_topic', reliable_qos)
        
        # 3. 尽力传输QoS配置（适合传感器数据）
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.best_effort_publisher = self.create_publisher(
            String, 'best_effort_topic', best_effort_qos)
        
        # 4. 持久化QoS配置（适合配置和参数）
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.transient_publisher = self.create_publisher(
            String, 'transient_topic', transient_qos)
        
        # 使用预定义的QoS配置
        from rclpy.qos import qos_profile_sensor_data
        self.sensor_publisher = self.create_publisher(
            String, 'sensor_topic', qos_profile_sensor_data)
        
        # 创建定时器，每秒发布一次消息
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        
        self.get_logger().info('QoS发布者已启动')
    
    def timer_callback(self):
        """
        定时器回调函数，发布消息到不同的主题
        """
        self.count += 1
        
        # 创建消息
        msg = String()
        timestamp = time.time()
        
        # 发布到默认主题
        msg.data = f'默认QoS消息 #{self.count} 时间戳: {timestamp}'
        self.default_publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')
        
        # 发布到可靠主题
        msg.data = f'可靠QoS消息 #{self.count} 时间戳: {timestamp}'
        self.reliable_publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')
        
        # 发布到尽力主题
        msg.data = f'尽力QoS消息 #{self.count} 时间戳: {timestamp}'
        self.best_effort_publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')
        
        # 发布到持久化主题
        msg.data = f'持久化QoS消息 #{self.count} 时间戳: {timestamp}'
        self.transient_publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')
        
        # 发布到传感器主题
        msg.data = f'传感器QoS消息 #{self.count} 时间戳: {timestamp}'
        self.sensor_publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisher()
    
    try:
        rclpy.spin(qos_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        qos_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
