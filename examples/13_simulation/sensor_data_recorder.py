#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import os
import cv2
from cv_bridge import CvBridge
import datetime
import message_filters
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SensorDataRecorder(Node):
    """
    传感器数据记录器，用于记录和分析仿真中的传感器数据
    """
    def __init__(self):
        super().__init__('sensor_data_recorder')
        
        # 创建QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建数据存储目录
        self.data_dir = os.path.join(os.getcwd(), 'sensor_data')
        os.makedirs(self.data_dir, exist_ok=True)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 创建订阅器
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/simple_robot/scan',
            self.laser_callback,
            qos_profile
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/simple_robot/image',
            self.image_callback,
            qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/simple_robot/odom',
            self.odom_callback,
            qos_profile
        )
        
        # 创建命令速度发布器
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/simple_robot/cmd_vel',
            qos_profile
        )
        
        # 初始化数据存储
        self.laser_data = []
        self.odom_data = []
        self.image_count = 0
        
        # 创建数据记录定时器
        self.record_timer = self.create_timer(5.0, self.save_data)
        
        # 创建可视化定时器
        self.viz_timer = self.create_timer(1.0, self.visualize_data)
        
        # 创建自动探索定时器
        self.explore_timer = self.create_timer(0.5, self.auto_explore)
        
        # 初始化探索状态
        self.explore_state = 'forward'
        self.state_timer = 0
        self.current_odom = None
        
        self.get_logger().info('传感器数据记录器已启动')
    
    def laser_callback(self, msg):
        """
        处理激光雷达数据
        """
        # 记录时间戳和范围数据
        timestamp = self.get_clock().now().to_msg()
        self.laser_data.append({
            'timestamp': timestamp,
            'ranges': msg.ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        })
        
        # 限制存储的数据量
        if len(self.laser_data) > 1000:
            self.laser_data = self.laser_data[-1000:]
    
    def image_callback(self, msg):
        """
        处理图像数据
        """
        try:
            # 每10帧保存一次图像
            if self.image_count % 10 == 0:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                image_path = os.path.join(self.data_dir, f'image_{timestamp}.jpg')
                cv2.imwrite(image_path, cv_image)
                
                # 在图像上添加时间戳
                cv2.putText(cv_image, timestamp, (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # 显示图像（如果在有GUI的环境中）
                # cv2.imshow("Camera View", cv_image)
                # cv2.waitKey(1)
            
            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
    
    def odom_callback(self, msg):
        """
        处理里程计数据
        """
        # 记录时间戳和位置数据
        timestamp = self.get_clock().now().to_msg()
        self.odom_data.append({
            'timestamp': timestamp,
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            }
        })
        
        # 更新当前里程计数据
        self.current_odom = msg
        
        # 限制存储的数据量
        if len(self.odom_data) > 1000:
            self.odom_data = self.odom_data[-1000:]
    
    def save_data(self):
        """
        保存收集的数据
        """
        try:
            # 保存激光数据
            if self.laser_data:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                laser_file = os.path.join(self.data_dir, f'laser_data_{timestamp}.npy')
                np.save(laser_file, self.laser_data)
                self.get_logger().info(f'激光数据已保存到 {laser_file}')
            
            # 保存里程计数据
            if self.odom_data:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                odom_file = os.path.join(self.data_dir, f'odom_data_{timestamp}.npy')
                np.save(odom_file, self.odom_data)
                self.get_logger().info(f'里程计数据已保存到 {odom_file}')
        except Exception as e:
            self.get_logger().error(f'保存数据时出错: {str(e)}')
    
    def visualize_data(self):
        """
        可视化最新的传感器数据
        """
        try:
            # 可视化激光数据
            if self.laser_data:
                latest_scan = self.laser_data[-1]
                ranges = np.array(latest_scan['ranges'])
                angles = np.linspace(
                    latest_scan['angle_min'], 
                    latest_scan['angle_max'], 
                    len(ranges)
                )
                
                # 将无效值替换为最大范围值
                ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0)
                
                # 计算x和y坐标
                x = ranges * np.cos(angles)
                y = ranges * np.sin(angles)
                
                # 在控制台打印简单的统计信息
                min_range = np.min(ranges)
                max_range = np.max(ranges[ranges < 10.0]) if np.any(ranges < 10.0) else 0
                avg_range = np.mean(ranges[ranges < 10.0]) if np.any(ranges < 10.0) else 0
                
                self.get_logger().info(
                    f'激光统计: 最小={min_range:.2f}m, 最大={max_range:.2f}m, 平均={avg_range:.2f}m'
                )
            
            # 可视化里程计数据
            if self.odom_data and len(self.odom_data) > 1:
                # 计算速度
                latest_odom = self.odom_data[-1]
                linear_speed = latest_odom['linear_velocity']['x']
                angular_speed = latest_odom['angular_velocity']['z']
                
                self.get_logger().info(
                    f'机器人速度: 线速度={linear_speed:.2f}m/s, 角速度={angular_speed:.2f}rad/s'
                )
                
                # 计算行驶距离
                if len(self.odom_data) > 10:
                    start_pos = self.odom_data[-10]['position']
                    end_pos = latest_odom['position']
                    distance = np.sqrt(
                        (end_pos['x'] - start_pos['x'])**2 + 
                        (end_pos['y'] - start_pos['y'])**2
                    )
                    
                    self.get_logger().info(f'最近行驶距离: {distance:.2f}m')
        except Exception as e:
            self.get_logger().error(f'可视化数据时出错: {str(e)}')
    
    def auto_explore(self):
        """
        自动探索环境
        """
        if not self.laser_data or not self.current_odom:
            return
        
        cmd_vel = Twist()
        
        # 获取最新的激光数据
        latest_scan = self.laser_data[-1]
        ranges = np.array(latest_scan['ranges'])
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0)
        
        # 检查前方是否有障碍物
        front_indices = list(range(len(ranges) - 30, len(ranges))) + list(range(0, 31))
        front_ranges = [ranges[i % len(ranges)] for i in front_indices]
        min_front_range = min(front_ranges)
        
        # 状态机控制
        if self.explore_state == 'forward':
            if min_front_range < 0.5:  # 前方有障碍物
                self.explore_state = 'turning'
                self.state_timer = 0
                # 决定转向方向
                left_ranges = ranges[len(ranges)//4:len(ranges)//2]
                right_ranges = ranges[len(ranges)//2:3*len(ranges)//4]
                if np.mean(left_ranges) > np.mean(right_ranges):
                    self.explore_state = 'turn_left'
                else:
                    self.explore_state = 'turn_right'
            else:
                cmd_vel.linear.x = 0.2
        
        elif self.explore_state == 'turn_left':
            cmd_vel.angular.z = 0.5
            self.state_timer += 1
            if self.state_timer > 10 or min_front_range > 1.0:  # 转弯约5秒或前方空旷
                self.explore_state = 'forward'
        
        elif self.explore_state == 'turn_right':
            cmd_vel.angular.z = -0.5
            self.state_timer += 1
            if self.state_timer > 10 or min_front_range > 1.0:  # 转弯约5秒或前方空旷
                self.explore_state = 'forward'
        
        # 发布速度命令
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    recorder = SensorDataRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        # 保存最终数据
        recorder.save_data()
        
        # 停止机器人
        stop_msg = Twist()
        recorder.cmd_vel_pub.publish(stop_msg)
        recorder.get_logger().info('已停止机器人')
        
        # 清理资源
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
