#!/usr/bin/env python3
# 基本标记发布器示例

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import time

class BasicMarkerPublisher(Node):
    def __init__(self):
        super().__init__('basic_marker_publisher')
        
        # 创建标记发布器
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'visualization_markers',
            10)
            
        # 创建定时器，每秒更新一次标记
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 初始化标记ID
        self.marker_id = 0
        
        self.get_logger().info('标记发布器已启动')
    
    def timer_callback(self):
        # 创建一个标记数组
        marker_array = MarkerArray()
        
        # 添加不同类型的标记
        marker_array.markers.append(self.create_arrow_marker())
        marker_array.markers.append(self.create_cube_marker())
        marker_array.markers.append(self.create_sphere_marker())
        marker_array.markers.append(self.create_cylinder_marker())
        marker_array.markers.append(self.create_text_marker())
        marker_array.markers.append(self.create_line_strip_marker())
        
        # 发布标记数组
        self.marker_pub.publish(marker_array)
        self.get_logger().info('已发布标记数组')
    
    def create_arrow_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'basic_shapes'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 设置箭头起点和终点
        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=1.0, y=0.0, z=0.0)
        ]
        
        # 设置尺寸
        marker.scale.x = 0.1  # 箭杆直径
        marker.scale.y = 0.2  # 箭头直径
        marker.scale.z = 0.0  # 不使用
        
        # 设置颜色（红色，半透明）
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
        
        # 设置持续时间（0表示永久）
        marker.lifetime.sec = 0
        
        return marker
    
    def create_cube_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'basic_shapes'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = 0.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.0
        
        # 设置方向（四元数）
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # 设置颜色（绿色，半透明）
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
        
        # 设置持续时间
        marker.lifetime.sec = 0
        
        return marker
    
    def create_sphere_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'basic_shapes'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = 0.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 0.0
        
        # 设置方向
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸（对于球体，所有维度应该相同）
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # 设置颜色（蓝色，半透明）
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
        
        # 设置持续时间
        marker.lifetime.sec = 0
        
        return marker
    
    def create_cylinder_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'basic_shapes'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # 设置方向
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸
        marker.scale.x = 0.5  # 直径
        marker.scale.y = 0.5  # 直径
        marker.scale.z = 1.0  # 高度
        
        # 设置颜色（黄色，半透明）
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
        
        # 设置持续时间
        marker.lifetime.sec = 0
        
        return marker
    
    def create_text_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'basic_shapes'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.5
        
        # 设置方向
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸（对于文本，z是文本高度）
        marker.scale.z = 0.3
        
        # 设置颜色（白色）
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # 设置文本内容
        marker.text = 'ROS2 可视化示例'
        
        # 设置持续时间
        marker.lifetime.sec = 0
        
        return marker
    
    def create_line_strip_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'basic_shapes'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 创建一个正弦波形状的线
        num_points = 20
        for i in range(num_points):
            x = float(i) / float(num_points) * 2.0
            y = math.sin(x * math.pi)
            marker.points.append(Point(x=x, y=y, z=0.0))
        
        # 设置线宽
        marker.scale.x = 0.05
        
        # 设置颜色（紫色）
        marker.color = ColorRGBA(r=0.8, g=0.2, b=0.8, a=1.0)
        
        # 设置持续时间
        marker.lifetime.sec = 0
        
        return marker

def main():
    rclpy.init()
    node = BasicMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('标记发布器被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
