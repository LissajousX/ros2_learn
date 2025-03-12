#!/usr/bin/env python3

"""
ROS2安全演示

这个脚本演示如何使用ROS2的安全特性，但要注意实际运行需要先配置好安全密钥和证书。
在运行此示例前，需要按照以下步骤设置安全环境：

1. 创建密钥库：
   ros2 security create_keystore ~/ros2_security_keystore

2. 为节点创建密钥：
   ros2 security create_key ~/ros2_security_keystore /security_demo/secure_talker
   ros2 security create_key ~/ros2_security_keystore /security_demo/secure_listener

3. 创建安全策略文件 security_policy.xml：
   (参见本目录下的security_policy.xml)

4. 应用安全策略：
   ros2 security create_permission ~/ros2_security_keystore /security_demo/secure_talker security_policy.xml
   ros2 security create_permission ~/ros2_security_keystore /security_demo/secure_listener security_policy.xml

5. 设置环境变量并运行节点：
   export ROS_SECURITY_KEYSTORE=~/ros2_security_keystore
   export ROS_SECURITY_ENABLE=true
   export ROS_SECURITY_STRATEGY=Enforce
   python3 security_demo.py --ros-args -e use_security:=true -e security_enclave:=/security_demo/secure_talker

   在另一个终端：
   export ROS_SECURITY_KEYSTORE=~/ros2_security_keystore
   export ROS_SECURITY_ENABLE=true
   export ROS_SECURITY_STRATEGY=Enforce
   python3 security_demo.py --ros-args -r __node:=listener -e use_security:=true -e security_enclave:=/security_demo/secure_listener
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse
import sys

class SecureNode(Node):
    """
    安全通信演示节点
    """
    def __init__(self, is_listener=False):
        # 根据角色确定节点名称
        node_name = 'listener' if is_listener else 'talker'
        super().__init__(node_name)
        
        self.is_listener = is_listener
        
        # 检查是否启用了安全特性
        from rclpy.utilities import get_parameter_value
        use_security = self.declare_parameter('use_security', False).value
        security_enclave = self.declare_parameter('security_enclave', '').value
        
        if use_security:
            self.get_logger().info(f'安全模式已启用，安全飞地: {security_enclave}')
        else:
            self.get_logger().info('安全模式未启用，使用普通通信')
        
        # 创建发布者或订阅者
        if is_listener:
            self.subscription = self.create_subscription(
                String,
                'secure_chatter',
                self.listener_callback,
                10)
            self.get_logger().info('安全监听器已启动，等待消息...')
        else:
            self.publisher = self.create_publisher(String, 'secure_chatter', 10)
            self.timer = self.create_timer(1.0, self.timer_callback)
            self.count = 0
            self.get_logger().info('安全发布者已启动，开始发送消息...')
    
    def listener_callback(self, msg):
        """
        接收消息的回调函数
        """
        self.get_logger().info(f'我收到: [{msg.data}]')
    
    def timer_callback(self):
        """
        定时发送消息的回调函数
        """
        msg = String()
        self.count += 1
        msg.data = f'安全消息 #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'发布: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    # 解析命令行参数，确定是发布者还是订阅者
    parser = argparse.ArgumentParser()
    parser.add_argument('--listener', action='store_true', help='作为监听器运行')
    
    # 使用sys.argv[1:]跳过ROS2特定的参数
    try:
        parsed_args, _ = parser.parse_known_args(sys.argv[1:])
    except SystemExit:
        parsed_args = parser.parse_args([])
    
    # 创建节点
    node = SecureNode(is_listener=parsed_args.listener)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
