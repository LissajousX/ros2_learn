import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult

class ParamCallbackNode(Node):
    def __init__(self):
        super().__init__('param_callback_node')
        
        # u4f7fu7528u53c2u6570u63cfu8ff0u7b26u63d0u4f9bu66f4u591au4fe1u606f
        my_str_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='u4e00u4e2au5b57u7b26u4e32u53c2u6570u793au4f8b'
        )
        
        # u58f0u660eu53c2u6570
        self.declare_parameter('callback_str_param', 'hello', my_str_descriptor)
        self.declare_parameter('callback_int_param', 10)
        
        # u6dfbu52a0u53c2u6570u53d8u5316u7684u56deu8c03
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # u521bu5efau5b9au65f6u5668
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('u53c2u6570u56deu8c03u8282u70b9u5df2u542fu52a8')
    
    def parameters_callback(self, params):
        """u5f53u53c2u6570u53d8u5316u65f6u8c03u7528u7684u56deu8c03u51fdu6570"""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            self.get_logger().info(f'u53c2u6570u53d8u5316: {param.name} = {param.value}')
            
            # u53efu4ee5u5728u8fd9u91ccu6dfbu52a0u53c2u6570u9a8cu8bc1u903bu8f91
            if param.name == 'callback_int_param' and param.value < 0:
                result.successful = False
                result.reason = 'u6574u6570u53c2u6570u4e0du80fdu4e3au8d1fu6570'
                break
        
        return result
    
    def timer_callback(self):
        # u83b7u53d6u5f53u524du53c2u6570u503c
        str_value = self.get_parameter('callback_str_param').value
        int_value = self.get_parameter('callback_int_param').value
        
        self.get_logger().info(f'u5f53u524du53c2u6570u503c: str={str_value}, int={int_value}')

def main():
    rclpy.init()
    node = ParamCallbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
