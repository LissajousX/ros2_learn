#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ParamNode(Node):
    def __init__(self, node_name='param_node'):
        """
        参数配置节点，用于演示启动文件中的参数配置
        """
        super().__init__(node_name)
        
        # 声明各种类型的参数
        self.declare_parameter(
            'string_param', 
            'default_value',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='字符串参数')
        )
        
        self.declare_parameter(
            'int_param', 
            42,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='整数参数')
        )
        
        self.declare_parameter(
            'double_param', 
            3.14,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='浮点数参数')
        )
        
        self.declare_parameter(
            'bool_param', 
            True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='布尔参数')
        )
        
        self.declare_parameter(
            'array_param', 
            [1, 2, 3],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY, description='整数数组参数')
        )
        
        # 创建定时器，定期输出参数值
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('参数节点已初始化')
    
    def timer_callback(self):
        """
        定时器回调函数，用于输出参数值
        """
        string_param = self.get_parameter('string_param').value
        int_param = self.get_parameter('int_param').value
        double_param = self.get_parameter('double_param').value
        bool_param = self.get_parameter('bool_param').value
        array_param = self.get_parameter('array_param').value
        
        self.get_logger().info('当前参数值:')
        self.get_logger().info(f'  string_param: {string_param}')
        self.get_logger().info(f'  int_param: {int_param}')
        self.get_logger().info(f'  double_param: {double_param}')
        self.get_logger().info(f'  bool_param: {bool_param}')
        self.get_logger().info(f'  array_param: {array_param}')

def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
