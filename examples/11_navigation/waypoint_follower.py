#!/usr/bin/env python3
# 路点跟随器示例

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # 创建一个回调组，允许并发执行
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建路点跟随动作客户端
        self.waypoint_client = ActionClient(
            self, 
            FollowWaypoints, 
            'follow_waypoints',
            callback_group=self.callback_group)
            
        # 等待动作服务器可用
        while not self.waypoint_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待路点跟随动作服务器...')
        
        self.get_logger().info('路点跟随动作服务器已连接！')
    
    def create_pose(self, x, y, theta=0.0, frame_id='map'):
        """创建一个姿态消息"""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 将角度转换为四元数
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return pose
    
    def follow_waypoints(self, waypoints):
        """跟随一系列路点"""
        # 创建动作目标
        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        
        self.get_logger().info(f'开始跟随{len(waypoints)}个路点')
        
        # 发送目标并等待结果
        self._send_goal_future = self.waypoint_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """目标响应回调函数"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('路点跟随目标被拒绝！')
            return
            
        self.get_logger().info('路点跟随目标被接受！')
        
        # 获取结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """获取结果回调函数"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # 成功
            missed_waypoints = result.missed_waypoints
            if len(missed_waypoints) == 0:
                self.get_logger().info('所有路点均已成功到达！')
            else:
                self.get_logger().warning(f'有{len(missed_waypoints)}个路点未能到达！')
                for i, wp_idx in enumerate(missed_waypoints):
                    self.get_logger().warning(f'未到达路点 {i+1}: 索引 {wp_idx}')
        else:
            self.get_logger().error(f'路点跟随失败，状态码: {status}')
    
    def feedback_callback(self, feedback_msg):
        """反馈回调函数"""
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        self.get_logger().info(f'当前正在前往第 {current_waypoint} 个路点')

def main():
    rclpy.init()
    follower = WaypointFollower()
    
    try:
        # 创建一系列路点
        waypoints = [
            follower.create_pose(1.0, 1.0, 0.0),     # 第一个路点
            follower.create_pose(2.0, 0.0, math.pi/2),  # 第二个路点
            follower.create_pose(0.0, 2.0, math.pi),    # 第三个路点
            follower.create_pose(0.0, 0.0, 3*math.pi/2)  # 第四个路点，回到起点
        ]
        
        # 开始跟随路点
        follower.follow_waypoints(waypoints)
        
        # 使用多线程执行器处理回调
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(follower)
        executor.spin()
    except KeyboardInterrupt:
        follower.get_logger().info('路点跟随被用户中断')
    finally:
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
