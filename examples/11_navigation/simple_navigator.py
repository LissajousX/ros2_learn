#!/usr/bin/env python3
# 简单导航节点示例

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # 创建一个回调组，允许并发执行
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建导航动作客户端
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.callback_group)
            
        # 等待动作服务器可用
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待导航动作服务器...')
        
        self.get_logger().info('导航动作服务器已连接！')
    
    def navigate_to(self, x, y, z, orientation_z, orientation_w, frame_id='map'):
        """导航到指定位置"""
        # 创建目标姿态消息
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        
        # 设置方向（四元数）
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = orientation_z
        goal_pose.pose.orientation.w = orientation_w
        
        # 创建动作目标
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        
        self.get_logger().info(f'开始导航到位置: x={x}, y={y}')
        
        # 发送目标并等待结果
        self._send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """目标响应回调函数"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝！')
            return
            
        self.get_logger().info('导航目标被接受！')
        
        # 获取结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """获取结果回调函数"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:
            self.get_logger().info('导航成功完成！')
        else:
            self.get_logger().error(f'导航失败，状态码: {status}')
    
    def feedback_callback(self, feedback_msg):
        """反馈回调函数"""
        feedback = feedback_msg.feedback
        # 计算到目标的距离
        current_pose = feedback.current_pose
        distance_remaining = feedback.distance_remaining
        
        # 每5秒打印一次反馈信息，避免日志过多
        current_time = time.time()
        if not hasattr(self, '_last_feedback_time') or current_time - self._last_feedback_time > 5.0:
            self.get_logger().info(f'当前位置: x={current_pose.pose.position.x:.2f}, y={current_pose.pose.position.y:.2f}')
            self.get_logger().info(f'剩余距离: {distance_remaining:.2f} 米')
            self._last_feedback_time = current_time

def main():
    rclpy.init()
    navigator = SimpleNavigator()
    
    try:
        # 导航到指定位置 (x, y, z, orientation_z, orientation_w)
        # 这里的坐标需要根据你的地图进行调整
        navigator.navigate_to(1.0, 1.0, 0.0, 0.0, 1.0)
        
        # 使用多线程执行器处理回调
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(navigator)
        executor.spin()
    except KeyboardInterrupt:
        navigator.get_logger().info('导航被用户中断')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
