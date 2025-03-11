#!/usr/bin/env python3
# 时间同步器示例 - 同步处理来自两个不同话题的消息

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import message_filters

class SyncSubscriber(Node):
    def __init__(self):
        super().__init__('sync_subscriber')
        
        # 创建两个订阅者
        image_sub = message_filters.Subscriber(self, Image, 'camera/image')
        pointcloud_sub = message_filters.Subscriber(self, PointCloud2, 'lidar/points')
        
        # 创建时间同步器，同步两个话题的消息
        # 参数10表示队列大小
        self.ts = message_filters.TimeSynchronizer([image_sub, pointcloud_sub], 10)
        self.ts.registerCallback(self.callback)
        
        self.get_logger().info('同步订阅者已启动')
    
    def callback(self, image_msg, pointcloud_msg):
        # 处理同步的消息
        self.get_logger().info(f'收到同步消息：')
        self.get_logger().info(f'图像时间戳: {image_msg.header.stamp.sec}.{image_msg.header.stamp.nanosec}')
        self.get_logger().info(f'点云时间戳: {pointcloud_msg.header.stamp.sec}.{pointcloud_msg.header.stamp.nanosec}')

def main():
    rclpy.init()
    node = SyncSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
