import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class BasicParamNode(Node):
    def __init__(self):
        super().__init__('basic_param_node')
        
        # 声明参数（名称、默认值、描述）
        self.declare_parameter('my_str_param', 'world')
        self.declare_parameter('my_int_param', 42)
        self.declare_parameter('my_double_param', 3.14)
        self.declare_parameter('my_bool_param', True)
        
        # 创建定时器，每秒读取并打印参数值
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('基础参数节点已启动')
        
    def timer_callback(self):
        # 获取参数值
        str_value = self.get_parameter('my_str_param').value
        int_value = self.get_parameter('my_int_param').value
        double_value = self.get_parameter('my_double_param').value
        bool_value = self.get_parameter('my_bool_param').value
        
        # 打印参数值
        self.get_logger().info(f'参数值: str={str_value}, int={int_value}, double={double_value}, bool={bool_value}')
        
        # 尝试动态修改参数
        new_int_value = int_value + 1
        self.set_parameters([Parameter('my_int_param', value=new_int_value)])
        self.get_logger().info(f'已将 my_int_param 更新为: {new_int_value}')

def main():
    rclpy.init()
    node = BasicParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
