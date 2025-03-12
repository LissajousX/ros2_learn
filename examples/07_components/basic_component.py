import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

# 组件必须继承Node类
class BasicComponent(Node):
    def __init__(self, node_name='basic_component'):
        super().__init__(node_name)
        
        # 声明参数
        self.declare_parameter('message', '组件消息')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        # 创建发布者
        self.publisher = self.create_publisher(String, 'component_topic', 10)
        
        # 获取参数
        self.message = self.get_parameter('message').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info(f'组件 {node_name} 已初始化')
    
    def timer_callback(self):
        # 创建消息
        msg = String()
        msg.data = f"{self.message} - {self.get_clock().now().to_msg().sec}"
        
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f'发布消息: {msg.data}')

# 这个函数是组件的入口点
def main(args=None):
    rclpy.init(args=args)
    
    # 创建组件
    component = BasicComponent()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(component)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        component.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# 这是组件接口，用于注册组件到ROS2组件系统
def create_basic_component(**kwargs):
    return BasicComponent(**kwargs)
