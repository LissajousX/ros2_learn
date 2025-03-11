import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParamGroupNode(Node):
    def __init__(self):
        super().__init__('param_group_node')
        
        # 声明分组参数
        self.declare_parameter('robot.name', 'my_robot')
        self.declare_parameter('robot.type', 'mobile')
        
        self.declare_parameter('motor.left.speed', 0.0)
        self.declare_parameter('motor.left.direction', 1)
        self.declare_parameter('motor.right.speed', 0.0)
        self.declare_parameter('motor.right.direction', 1)
        
        self.declare_parameter('sensor.front.enabled', True)
        self.declare_parameter('sensor.rear.enabled', False)
        
        # 创建定时器
        self.timer = self.create_timer(3.0, self.timer_callback)
        
        self.get_logger().info('参数组节点已启动')
    
    def timer_callback(self):
        # 获取机器人信息
        robot_name = self.get_parameter('robot.name').value
        robot_type = self.get_parameter('robot.type').value
        
        # 获取电机参数
        left_speed = self.get_parameter('motor.left.speed').value
        right_speed = self.get_parameter('motor.right.speed').value
        
        # 获取传感器状态
        front_sensor = self.get_parameter('sensor.front.enabled').value
        rear_sensor = self.get_parameter('sensor.rear.enabled').value
        
        # 打印信息
        self.get_logger().info(f'机器人: {robot_name} ({robot_type})')
        self.get_logger().info(f'电机速度: 左={left_speed}, 右={right_speed}')
        self.get_logger().info(f'传感器: 前={front_sensor}, 后={rear_sensor}')
        
        # 模拟更新电机速度
        self.set_parameters([
            Parameter('motor.left.speed', value=left_speed + 0.1),
            Parameter('motor.right.speed', value=right_speed + 0.1)
        ])

def main():
    rclpy.init()
    node = ParamGroupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
