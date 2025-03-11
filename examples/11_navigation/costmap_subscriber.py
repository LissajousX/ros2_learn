#!/usr/bin/env python3
# 代价地图订阅者示例

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

class CostmapSubscriber(Node):
    def __init__(self):
        super().__init__('costmap_subscriber')
        
        # 订阅全局代价地图
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.global_costmap_callback,
            10)
            
        # 订阅局部代价地图
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.local_costmap_callback,
            10)
            
        self.get_logger().info('代价地图订阅者已启动')
        
        # 创建用于可视化的颜色映射
        self.cmap = ListedColormap(['white', 'lightgrey', 'grey', 'darkgrey', 'black'])
        
        # 保存最新的代价地图数据
        self.latest_global_costmap = None
        self.latest_local_costmap = None
    
    def global_costmap_callback(self, msg):
        """全局代价地图回调函数"""
        self.get_logger().info('收到全局代价地图更新')
        self.latest_global_costmap = msg
        self.process_costmap(msg, 'global')
    
    def local_costmap_callback(self, msg):
        """局部代价地图回调函数"""
        self.get_logger().info('收到局部代价地图更新')
        self.latest_local_costmap = msg
        self.process_costmap(msg, 'local')
    
    def process_costmap(self, costmap_msg, costmap_type):
        """处理代价地图数据"""
        # 提取元数据
        resolution = costmap_msg.info.resolution  # 地图分辨率（米/像素）
        width = costmap_msg.info.width           # 地图宽度（像素）
        height = costmap_msg.info.height         # 地图高度（像素）
        origin_x = costmap_msg.info.origin.position.x  # 地图原点X坐标
        origin_y = costmap_msg.info.origin.position.y  # 地图原点Y坐标
        
        # 将一维数组转换为二维数组
        costmap_data = np.array(costmap_msg.data).reshape(height, width)
        
        # 打印代价地图信息
        self.get_logger().info(f'{costmap_type} 代价地图信息:')
        self.get_logger().info(f'  分辨率: {resolution} 米/像素')
        self.get_logger().info(f'  尺寸: {width} x {height} 像素')
        self.get_logger().info(f'  物理尺寸: {width*resolution:.2f} x {height*resolution:.2f} 米')
        self.get_logger().info(f'  原点: ({origin_x:.2f}, {origin_y:.2f})')
        
        # 统计代价值分布
        free_cells = np.sum(costmap_data == 0)  # 自由空间
        occupied_cells = np.sum(costmap_data == 100)  # 障碍物
        unknown_cells = np.sum(costmap_data == -1)  # 未知区域
        
        self.get_logger().info(f'  自由空间单元格: {free_cells} ({free_cells/(width*height)*100:.1f}%)')
        self.get_logger().info(f'  障碍物单元格: {occupied_cells} ({occupied_cells/(width*height)*100:.1f}%)')
        self.get_logger().info(f'  未知区域单元格: {unknown_cells} ({unknown_cells/(width*height)*100:.1f}%)')
    
    def visualize_costmap(self, costmap_type='global'):
        """可视化代价地图"""
        # 选择要可视化的代价地图
        costmap_msg = self.latest_global_costmap if costmap_type == 'global' else self.latest_local_costmap
        
        if costmap_msg is None:
            self.get_logger().error(f'没有可用的{costmap_type}代价地图数据')
            return
        
        # 提取元数据
        resolution = costmap_msg.info.resolution
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        origin_x = costmap_msg.info.origin.position.x
        origin_y = costmap_msg.info.origin.position.y
        
        # 将一维数组转换为二维数组
        costmap_data = np.array(costmap_msg.data).reshape(height, width)
        
        # 将-1（未知）映射为一个特定值，以便可视化
        costmap_data_viz = costmap_data.copy()
        costmap_data_viz[costmap_data_viz == -1] = 50  # 未知区域映射为灰色
        
        # 创建图形
        plt.figure(figsize=(10, 8))
        plt.imshow(costmap_data_viz, cmap='gray', origin='lower', 
                  extent=[origin_x, origin_x + width * resolution, 
                          origin_y, origin_y + height * resolution])
        plt.colorbar(label='代价值')
        plt.title(f'{costmap_type.capitalize()} 代价地图')
        plt.xlabel('X (米)')
        plt.ylabel('Y (米)')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.show()

def main():
    rclpy.init()
    node = CostmapSubscriber()
    
    try:
        # 收集一段时间的代价地图数据
        rclpy.spin_once(node, timeout_sec=10.0)
        
        # 如果收到了代价地图数据，可视化它
        if node.latest_global_costmap is not None:
            node.visualize_costmap('global')
        if node.latest_local_costmap is not None:
            node.visualize_costmap('local')
        
        # 继续运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
