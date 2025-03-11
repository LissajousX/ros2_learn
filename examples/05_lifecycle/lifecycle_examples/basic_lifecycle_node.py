import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State

class BasicLifecycleNode(LifecycleNode):
    def __init__(self):
        # 生命周期节点需要一个节点名
        super().__init__('basic_lifecycle_node')
        
        # 在未配置状态下，我们只能创建变量，但不能创建订阅者、发布者、服务等
        self.timer = None
        self.counter = 0
        
        # 在ROS2 Humble中，生命周期节点初始状态为未配置
        self.get_logger().info('生命周期节点创建完成，当前状态: 未配置')
    
    def on_configure(self, state):
        """当节点从未配置状态转换到非活动状态时调用"""
        self.get_logger().info('正在配置节点...')
        
        # 在这里创建参数
        self.declare_parameter('lifecycle_param', 'default_value')
        
        # 返回成功表示配置完成
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        """当节点从非活动状态转换到活动状态时调用"""
        self.get_logger().info('正在激活节点...')
        
        # 创建定时器，在活动状态下执行定期任务
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 返回成功表示激活完成
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        """当节点从活动状态转换到非活动状态时调用"""
        self.get_logger().info('正在停用节点...')
        
        # 销毁定时器
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        
        # 返回成功表示停用完成
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        """当节点从非活动状态转换到未配置状态时调用"""
        self.get_logger().info('正在清理节点...')
        
        # 重置计数器
        self.counter = 0
        
        # 返回成功表示清理完成
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state):
        """当节点关闭时调用，可以从任何状态转换到关闭状态"""
        self.get_logger().info('正在关闭节点...')
        
        # 销毁定时器（如果存在）
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        
        # 返回成功表示关闭完成
        return TransitionCallbackReturn.SUCCESS
    
    def timer_callback(self):
        """定时器回调函数，只在活动状态下执行"""
        # 获取参数值
        param_value = self.get_parameter('lifecycle_param').value
        
        # 增加计数器并打印信息
        self.counter += 1
        self.get_logger().info(f'节点活动中: 计数={self.counter}, 参数={param_value}')

def main():
    rclpy.init()
    
    # 创建生命周期节点
    node = BasicLifecycleNode()
    
    try:
        # 运行节点
        # 注意：生命周期节点初始状态为未配置，需要通过外部命令或服务来改变状态
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理节点
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
