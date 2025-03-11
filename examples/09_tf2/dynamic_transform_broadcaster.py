#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_frame_publisher')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_dynamic_transform)  # 10Hz
        self.angle = 0.0
        
    def publish_dynamic_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'rotating_link'
        
        # 计算旋转位置
        radius = 0.5  # 旋转半径
        transform.transform.translation.x = radius * math.cos(self.angle)
        transform.transform.translation.y = radius * math.sin(self.angle)
        transform.transform.translation.z = 0.0
        
        # 设置旋转（四元数）
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.angle / 2.0)
        transform.transform.rotation.w = math.cos(self.angle / 2.0)
        
        # 发布动态变换
        self.broadcaster.sendTransform(transform)
        
        # 更新角度
        self.angle += 0.1
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

def main():
    rclpy.init()
    node = DynamicFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
