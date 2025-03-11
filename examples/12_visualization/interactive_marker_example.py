#!/usr/bin/env python3
# 交互式标记示例

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
import math

class InteractiveMarkerExample(Node):
    def __init__(self):
        super().__init__('interactive_marker_example')
        
        # 创建交互式标记服务器
        self.server_name = 'simple_marker_server'
        self.interactive_marker_pub = self.create_publisher(
            InteractiveMarker,
            f'{self.server_name}/update',
            10)
        
        # 创建反馈订阅者
        self.feedback_sub = self.create_subscription(
            InteractiveMarkerFeedback,
            f'{self.server_name}/feedback',
            self.process_feedback,
            10)
        
        # 初始化标记
        self.create_interactive_markers()
        
        self.get_logger().info('交互式标记示例已启动')
    
    def create_interactive_markers(self):
        # 创建一个简单的交互式标记
        self.create_simple_marker()
        
        # 创建一个带有多个控制的标记
        self.create_6dof_marker()
        
        # 创建一个菜单标记
        self.create_menu_marker()
    
    def create_simple_marker(self):
        # 创建一个基本的交互式标记
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = 'simple_marker'
        int_marker.description = '简单交互标记'
        
        # 设置标记位置
        int_marker.pose.position = Point(x=0.0, y=0.0, z=1.0)
        
        # 创建一个球体标记作为视觉表示
        marker = self.create_sphere_marker()
        
        # 创建控制元素
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        # 添加移动控制
        control = InteractiveMarkerControl()
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.y = 1.0
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)
        
        # 发布交互式标记
        self.interactive_marker_pub.publish(int_marker)
    
    def create_6dof_marker(self):
        # 创建一个具有6自由度控制的交互式标记
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = '6dof_marker'
        int_marker.description = '6自由度标记'
        
        # 设置标记位置
        int_marker.pose.position = Point(x=2.0, y=0.0, z=1.0)
        
        # 创建一个立方体标记作为视觉表示
        marker = self.create_cube_marker()
        
        # 创建控制元素
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        # 添加旋转控制
        control = InteractiveMarkerControl()
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.y = 1.0
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)
        
        # 添加移动控制
        control = InteractiveMarkerControl()
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.y = 1.0
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)
        
        # 发布交互式标记
        self.interactive_marker_pub.publish(int_marker)
    
    def create_menu_marker(self):
        # 创建一个带有菜单的交互式标记
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = 'menu_marker'
        int_marker.description = '菜单标记'
        
        # 设置标记位置
        int_marker.pose.position = Point(x=0.0, y=2.0, z=1.0)
        
        # 创建一个圆柱体标记作为视觉表示
        marker = self.create_cylinder_marker()
        
        # 创建控制元素
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        # 添加菜单控制
        control = InteractiveMarkerControl()
        control.name = 'menu_control'
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description = '菜单'
        int_marker.controls.append(control)
        
        # 添加菜单项
        int_marker.menu_entries = [
            {'id': 1, 'parent_id': 0, 'title': '改变颜色', 'command': 'COLOR_CHANGE'},
            {'id': 2, 'parent_id': 1, 'title': '红色', 'command': 'COLOR_RED'},
            {'id': 3, 'parent_id': 1, 'title': '绿色', 'command': 'COLOR_GREEN'},
            {'id': 4, 'parent_id': 1, 'title': '蓝色', 'command': 'COLOR_BLUE'},
            {'id': 5, 'parent_id': 0, 'title': '改变形状', 'command': 'SHAPE_CHANGE'},
            {'id': 6, 'parent_id': 5, 'title': '立方体', 'command': 'SHAPE_CUBE'},
            {'id': 7, 'parent_id': 5, 'title': '球体', 'command': 'SHAPE_SPHERE'},
            {'id': 8, 'parent_id': 5, 'title': '圆柱体', 'command': 'SHAPE_CYLINDER'}
        ]
        
        # 发布交互式标记
        self.interactive_marker_pub.publish(int_marker)
    
    def create_sphere_marker(self):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker
    
    def create_cube_marker(self):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker
    
    def create_cylinder_marker(self):
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker
    
    def process_feedback(self, feedback):
        # 处理交互式标记的反馈
        marker_name = feedback.marker_name
        control_name = feedback.control_name
        interaction_type = feedback.event_type
        position = feedback.pose.position
        
        # 记录反馈信息
        self.get_logger().info(f'收到标记 {marker_name} 的反馈:')
        self.get_logger().info(f'  控制: {control_name}')
        self.get_logger().info(f'  交互类型: {interaction_type}')
        self.get_logger().info(f'  位置: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})')
        
        # 处理菜单命令
        if interaction_type == InteractiveMarkerFeedback.MENU_SELECT:
            command = feedback.menu_entry_id
            self.get_logger().info(f'  菜单选择: {command}')
            
            # 根据菜单命令更新标记
            # 注意：在实际应用中，这里需要更新标记并重新发布

def main():
    rclpy.init()
    node = InteractiveMarkerExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('交互式标记示例被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
