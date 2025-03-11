#!/usr/bin/env python3
# TF2过滤器示例 - 等待特定的坐标变换可用后再处理消息

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import message_filters
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfFilterExample(Node):
    def __init__(self):
        super().__init__('tf_filter_example')
        
        # 创建TF2缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建点云订阅者
        self.cloud_sub = message_filters.Subscriber(self, PointCloud2, 'lidar/points')
        
        # 创建TF2过滤器，等待从lidar_frame到base_link的变换可用
        self.tf_filter = tf2_ros.MessageFilter(
            self.cloud_sub, self.tf_buffer, 'base_link', 10, self)
        self.tf_filter.registerCallback(self.cloud_callback)
        
        self.get_logger().info('TF2过滤器示例已启动')
    
    def cloud_callback(self, cloud_msg):
        try:
            # 获取从点云帧到base_link的变换
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp)
            
            # 处理点云数据和变换
            self.get_logger().info(f'收到点云消息，帧ID: {cloud_msg.header.frame_id}')
            self.get_logger().info(f'变换可用: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}')
            
            # 这里可以使用tf2_sensor_msgs库来变换点云数据
            # transformed_cloud = tf2_sensor_msgs.do_transform_cloud(cloud_msg, transform)
            
        except Exception as e:
            self.get_logger().error(f'处理点云消息时出错: {str(e)}')

def main():
    rclpy.init()
    node = TfFilterExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
