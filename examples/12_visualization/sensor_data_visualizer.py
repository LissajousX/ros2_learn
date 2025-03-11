#!/usr/bin/env python3
# 传感器数据可视化示例

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import numpy as np

class SensorDataVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_data_visualizer')
        
        # 订阅激光雷达数据
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        # 创建激光雷达可视化发布器
        self.laser_marker_pub = self.create_publisher(
            MarkerArray,
            'laser_visualization',
            10)
        
        # 订阅点云数据（如果有的话）
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points',
            self.pointcloud_callback,
            10)
        
        # 订阅图像数据（如果有的话）
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 初始化标记ID
        self.marker_id = 0
        
        self.get_logger().info('传感器数据可视化器已启动')
    
    def laser_callback(self, msg):
        """处理激光雷达数据并创建可视化标记"""
        self.get_logger().info('收到激光雷达数据')
        
        # 创建标记数组
        marker_array = MarkerArray()
        
        # 清除之前的标记
        marker_array.markers.append(self.create_delete_marker())
        
        # 创建激光雷达点标记
        marker_array.markers.append(self.create_laser_points_marker(msg))
        
        # 创建激光雷达轮廓标记
        marker_array.markers.append(self.create_laser_contour_marker(msg))
        
        # 创建障碍物标记
        marker_array.markers.extend(self.create_obstacle_markers(msg))
        
        # 发布标记数组
        self.laser_marker_pub.publish(marker_array)
    
    def create_delete_marker(self):
        """创建一个用于清除所有先前标记的标记"""
        marker = Marker()
        marker.header.frame_id = 'base_link'  # 激光雷达的参考坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'laser_visualization'
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker
    
    def create_laser_points_marker(self, scan_msg):
        """创建显示激光雷达所有点的标记"""
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'laser_visualization'
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # 设置点的大小
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        
        # 设置点的颜色（绿色）
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
        
        # 将激光雷达数据转换为点
        angle = scan_msg.angle_min
        for range_value in scan_msg.ranges:
            # 检查范围是否有效
            if range_value >= scan_msg.range_min and range_value <= scan_msg.range_max:
                # 计算点的笛卡尔坐标
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                marker.points.append(Point(x=x, y=y, z=0.0))
            
            angle += scan_msg.angle_increment
        
        return marker
    
    def create_laser_contour_marker(self, scan_msg):
        """创建显示激光雷达轮廓的标记"""
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'laser_visualization'
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 设置线的宽度
        marker.scale.x = 0.03
        
        # 设置线的颜色（蓝色）
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        # 将激光雷达数据转换为点
        angle = scan_msg.angle_min
        for range_value in scan_msg.ranges:
            # 检查范围是否有效
            if range_value >= scan_msg.range_min and range_value <= scan_msg.range_max:
                # 计算点的笛卡尔坐标
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                marker.points.append(Point(x=x, y=y, z=0.0))
            else:
                # 对于无效范围，添加一个距离最小值的点
                x = scan_msg.range_min * math.cos(angle)
                y = scan_msg.range_min * math.sin(angle)
                marker.points.append(Point(x=x, y=y, z=0.0))
            
            angle += scan_msg.angle_increment
        
        # 闭合轮廓
        if len(marker.points) > 0:
            marker.points.append(marker.points[0])
        
        return marker
    
    def create_obstacle_markers(self, scan_msg):
        """创建显示探测到的障碍物的标记"""
        markers = []
        
        # 简单的障碍物检测算法
        # 这里使用一个非常简单的方法：如果激光距离小于阈值，则认为是障碍物
        threshold = 1.0  # 障碍物距离阈值（米）
        min_points = 3   # 最小点数构成障碍物
        
        # 将激光雷达数据转换为点
        points = []
        angle = scan_msg.angle_min
        for range_value in scan_msg.ranges:
            # 检查范围是否有效u4e14u5c0fu4e8eu9608u503c
            if (range_value >= scan_msg.range_min and 
                range_value <= scan_msg.range_max and 
                range_value < threshold):
                # 计算点的笛卡尔坐标
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                points.append((x, y))
            
            angle += scan_msg.angle_increment
        
        # 简化的聚类算法，将相近的点分组
        clusters = []
        current_cluster = []
        
        for i in range(len(points)):
            if not current_cluster:
                current_cluster.append(points[i])
            else:
                # 计算与前一个点的距离
                prev_x, prev_y = current_cluster[-1]
                curr_x, curr_y = points[i]
                dist = math.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)
                
                if dist < 0.2:  # 如果距离小于20cm，则属于同一类
                    current_cluster.append(points[i])
                else:
                    # 如果当前类别有足够的点，则保存它
                    if len(current_cluster) >= min_points:
                        clusters.append(current_cluster)
                    # 开始新的类别
                    current_cluster = [points[i]]
        
        # 添加最后一个类别（如果有足够的点）
        if len(current_cluster) >= min_points:
            clusters.append(current_cluster)
        
        # 为每个聚类创建标记
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = scan_msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i + 10  # 避免与其他标记ID冲突
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 计算聚类的中心点
            center_x = sum(p[0] for p in cluster) / len(cluster)
            center_y = sum(p[1] for p in cluster) / len(cluster)
            
            # 计算聚类的半径（使用到中心最远的点的距离）
            radius = max(math.sqrt((p[0] - center_x)**2 + (p[1] - center_y)**2) for p in cluster)
            radius = max(radius, 0.1)  # 最小半径
            
            # 设置标记位置和大小
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.0
            marker.scale.x = radius * 2
            marker.scale.y = radius * 2
            marker.scale.z = 0.1  # 高度
            
            # 设置颜色（红色）
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            
            # 添加到标记列表
            markers.append(marker)
        
        return markers
    
    def pointcloud_callback(self, msg):
        """处理点云数据"""
        # 注意：处理点云数据需要使用额外的库，如sensor_msgs_py
        # 这里只是一个简单的占位符
        self.get_logger().info('收到点云数据')
        # 实际实现中应该将点云转换为可视化标记
    
    def image_callback(self, msg):
        """处理图像数据"""
        # 注意：图像数据已经可以直接在RViz2中显示
        # 这里只是一个简单的占位符
        self.get_logger().info('收到图像数据')

def main():
    rclpy.init()
    node = SensorDataVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('传感器数据可视化器被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
