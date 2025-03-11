#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()
        
    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'
        
        # 设置平移（单位：米）
        transform.transform.translation.x = 0.1
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.2
        
        # 设置旋转（四元数）
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # 发布静态变换
        self.broadcaster.sendTransform(transform)
        self.get_logger().info('已发布从base_link到camera_link的静态变换')

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
