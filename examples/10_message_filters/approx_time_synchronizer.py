#!/usr/bin/env python3
# 近似时间同步器示例 - 允许消息之间存在一定的时间误差

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu
import message_filters

class ApproxSyncSubscriber(Node):
    def __init__(self):
        super().__init__('approx_sync_subscriber')
        
        # 创建三个订阅者
        image_sub = message_filters.Subscriber(self, Image, 'camera/image')
        pointcloud_sub = message_filters.Subscriber(self, PointCloud2, 'lidar/points')
        imu_sub = message_filters.Subscriber(self, Imu, 'imu/data')
        
        # 创建近似时间同步器
        # 参数：订阅者列表，队列大小，最大时间误差（秒）
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, pointcloud_sub, imu_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        
        self.get_logger().info('近似时间同步订阅者已启动')
    
    def callback(self, image_msg, pointcloud_msg, imu_msg):
        # 处理同步的消息
        self.get_logger().info('收到近似同步的消息：')
        self.get_logger().info(f'图像时间戳: {image_msg.header.stamp.sec}.{image_msg.header.stamp.nanosec}')
        self.get_logger().info(f'点云时间戳: {pointcloud_msg.header.stamp.sec}.{pointcloud_msg.header.stamp.nanosec}')
        self.get_logger().info(f'IMU时间戳: {imu_msg.header.stamp.sec}.{imu_msg.header.stamp.nanosec}')
        
        # 计算时间差异
        image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        pointcloud_time = pointcloud_msg.header.stamp.sec + pointcloud_msg.header.stamp.nanosec * 1e-9
        imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        
        self.get_logger().info(f'图像和点云时间差: {abs(image_time - pointcloud_time):.6f} 秒')
        self.get_logger().info(f'图像和IMU时间差: {abs(image_time - imu_time):.6f} 秒')
        self.get_logger().info(f'点云和IMU时间差: {abs(pointcloud_time - imu_time):.6f} 秒')

def main():
    rclpy.init()
    node = ApproxSyncSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
