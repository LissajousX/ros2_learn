#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import threading
import time

class QoSCompatibilityTest(Node):
    """
    测试不同QoS配置之间的兼容性
    """
    def __init__(self):
        super().__init__('qos_compatibility_test')
        
        # 创建不同QoS配置的发布者
        
        # 可靠传输发布者
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.reliable_publisher = self.create_publisher(
            String, 'compatibility/reliable', reliable_qos)
        
        # 尽力传输发布者
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.best_effort_publisher = self.create_publisher(
            String, 'compatibility/best_effort', best_effort_qos)
        
        # 持久化发布者
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.transient_publisher = self.create_publisher(
            String, 'compatibility/transient', transient_qos)
        
        # 创建不同QoS配置的订阅者
        
        # 1. 兼容的订阅者
        self.reliable_sub_reliable = self.create_subscription(
            String,
            'compatibility/reliable',
            lambda msg: self.get_logger().info(f'兼容订阅者收到: {msg.data}'),
            reliable_qos)
        
        self.best_effort_sub_best_effort = self.create_subscription(
            String,
            'compatibility/best_effort',
            lambda msg: self.get_logger().info(f'兼容订阅者收到: {msg.data}'),
            best_effort_qos)
        
        self.transient_sub_transient = self.create_subscription(
            String,
            'compatibility/transient',
            lambda msg: self.get_logger().info(f'兼容订阅者收到: {msg.data}'),
            transient_qos)
        
        # 2. 不兼容的订阅者
        # 尝试用RELIABLE订阅BEST_EFFORT主题
        self.incompatible_sub1 = self.create_subscription(
            String,
            'compatibility/best_effort',
            lambda msg: self.get_logger().info(f'不兼容订阅者1收到: {msg.data}'),
            reliable_qos)
        
        # 尝试用BEST_EFFORT订阅RELIABLE主题
        self.incompatible_sub2 = self.create_subscription(
            String,
            'compatibility/reliable',
            lambda msg: self.get_logger().info(f'不兼容订阅者2收到: {msg.data}'),
            best_effort_qos)
        
        # 尝试用VOLATILE订阅TRANSIENT_LOCAL主题
        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.incompatible_sub3 = self.create_subscription(
            String,
            'compatibility/transient',
            lambda msg: self.get_logger().info(f'不兼容订阅者3收到: {msg.data}'),
            volatile_qos)
        
        # 创建发布线程
        self.stop_flag = False
        self.publish_thread = threading.Thread(target=self.publish_messages)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        self.get_logger().info('QoS兼容性测试已启动')
    
    def publish_messages(self):
        """
        在单独的线程中发布消息
        """
        count = 0
        while not self.stop_flag:
            count += 1
            
            # 创建消息
            reliable_msg = String()
            reliable_msg.data = f'可靠消息 #{count}'
            
            best_effort_msg = String()
            best_effort_msg.data = f'尽力消息 #{count}'
            
            transient_msg = String()
            transient_msg.data = f'持久化消息 #{count}'
            
            # 发布消息
            self.reliable_publisher.publish(reliable_msg)
            self.best_effort_publisher.publish(best_effort_msg)
            self.transient_publisher.publish(transient_msg)
            
            self.get_logger().info(f'发布了3种QoS消息 #{count}')
            
            # 等待一秒
            time.sleep(1.0)
    
    def __del__(self):
        self.stop_flag = True
        if hasattr(self, 'publish_thread'):
            self.publish_thread.join()

def main(args=None):
    rclpy.init(args=args)
    qos_test = QoSCompatibilityTest()
    
    try:
        rclpy.spin(qos_test)
    except KeyboardInterrupt:
        pass
    finally:
        qos_test.stop_flag = True
        qos_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
