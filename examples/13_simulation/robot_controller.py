#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class RobotController(Node):
    """
    简单的机器人控制器，实现基本的避障功能
    """
    def __init__(self):
        super().__init__('robot_controller')
        
        # 创建QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建速度发布器
        self.vel_pub = self.create_publisher(
            Twist, 
            '/simple_robot/cmd_vel', 
            qos_profile
        )
        
        # 创建激光扫描订阅器
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/simple_robot/scan',
            self.scan_callback,
            qos_profile
        )
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 初始化变量
        self.scan_data = None
        self.obstacle_detected = False
        self.obstacle_direction = 0.0
        
        self.get_logger().info('机器人控制器已启动')
    
    def scan_callback(self, msg):
        """
        处理激光扫描数据
        """
        self.scan_data = msg
        
        # 检测障碍物
        ranges = np.array(msg.ranges)
        # 将无效值（inf或nan）替换为最大范围值
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)
        
        # 定义安全距离
        safe_distance = 0.5
        
        # 检查前方区域是否有障碍物（-30度到30度）
        front_indices = np.arange(len(ranges) - 30, len(ranges)) % len(ranges)
        front_indices = np.append(front_indices, np.arange(0, 31))
        front_ranges = ranges[front_indices]
        
        if np.any(front_ranges < safe_distance):
            self.obstacle_detected = True
            # 找出最近障碍物的方向
            min_idx = front_indices[np.argmin(front_ranges)]
            self.obstacle_direction = (min_idx * msg.angle_increment) + msg.angle_min
        else:
            self.obstacle_detected = False
    
    def control_loop(self):
        """
        主控制循环
        """
        if self.scan_data is None:
            # 还没有收到激光数据，保持静止
            cmd_vel = Twist()
            self.vel_pub.publish(cmd_vel)
            return
        
        cmd_vel = Twist()
        
        if self.obstacle_detected:
            # 有障碍物，执行避障
            self.get_logger().debug(f'检测到障碍物，方向: {self.obstacle_direction}')
            
            # 根据障碍物方向决定转向方向
            if self.obstacle_direction > 0:  # 障碍物在右侧
                cmd_vel.angular.z = 0.5  # 左转
            else:  # 障碍物在左侧
                cmd_vel.angular.z = -0.5  # 右转
            
            # 减速或后退
            if min(self.scan_data.ranges) < 0.3:  # 非常近的障碍物
                cmd_vel.linear.x = -0.1  # 后退
            else:
                cmd_vel.linear.x = 0.05  # 慢速前进
        else:
            # 无障碍物，正常前进
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.0
        
        # 发布速度命令
        self.vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        stop_msg = Twist()
        controller.vel_pub.publish(stop_msg)
        controller.get_logger().info('已停止机器人')
        
        # 清理资源
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
