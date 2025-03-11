import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('斐波那契动作服务器已启动')
    
    def goal_callback(self, goal_request):
        """接受或拒绝目标请求的回调函数。"""
        self.get_logger().info(f'收到目标请求: 计算{goal_request.order}个斐波那契数')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """处理取消请求的回调函数。"""
        self.get_logger().info(f'收到取消请求')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'开始执行目标: 计算斐波那契数列，序列长度为 {goal_handle.request.order}')
        
        # 创建反馈和结果消息
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()
        
        # 初始化斐波那契数列
        feedback_msg.sequence = [0, 1]
        
        # 发布初始反馈
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'发布初始反馈: {feedback_msg.sequence}')
        
        # 计算斐波那契数列
        for i in range(1, goal_handle.request.order):
            # 首先检查是否有取消请求
            if goal_handle.is_cancel_requested:
                self.get_logger().info('主循环中检测到取消请求')
                goal_handle.canceled()
                self.get_logger().info('目标已被取消')
                return Fibonacci.Result()
                
            # 计算下一个斐波那契数
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            # 发布反馈
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'发布反馈: {feedback_msg.sequence}')
            
            # 模拟长时间运行的任务
            self.get_logger().info(f'处理第 {i+1} 个数，等待1秒...')
            
            # 将1秒分成10个小段，每段后检查取消状态
            for j in range(10):
                time.sleep(0.1)
                # 频繁检查是否有取消请求
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f'在等待过程中检测到取消请求 (第 {j+1}/10 段)')
                    goal_handle.canceled()
                    self.get_logger().info('目标已被取消')
                    return Fibonacci.Result()
        
        # 设置结果
        result.sequence = feedback_msg.sequence
        goal_handle.succeed()
        self.get_logger().info('目标成功完成')
        
        return result

def main():
    rclpy.init()
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()