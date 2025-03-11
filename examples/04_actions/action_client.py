import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci
import sys
import time
import threading

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self._goal_handle = None
        self._goal_future = None
        self._get_result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._cancel_completed = False

    def send_goal(self, order):
        self.get_logger().info('等待动作服务器...')
        self._action_client.wait_for_server()
        
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self.get_logger().info(f'发送目标请求：计算{order}个斐波那契数')
        
        self._goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        
        self._goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        
        if not self._goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return
            
        self.get_logger().info('目标被接受')
        
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'结果状态: {status}')
        self.get_logger().info(f'结果: {result.sequence}')
        
        # 如果我们请求了取消但还没有完成，等待取消完成
        if self._cancel_requested and not self._cancel_completed:
            self.get_logger().info('等待取消完成...')
            # 等待一些时间以确保取消完成
            time.sleep(1)
        
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'收到反馈: {feedback.sequence}')
    
    def cancel_goal(self):
        if self._goal_handle:
            self.get_logger().info('准备取消目标')
            self._cancel_requested = True
            
            # 确保目标句柄有效且目标已被接受
            if self._goal_handle.accepted:
                self.get_logger().info('目标已被接受，发送取消请求')
                self._cancel_future = self._goal_handle.cancel_goal_async()
                self._cancel_future.add_done_callback(self.cancel_done)
            else:
                self.get_logger().warn('无法取消目标：目标尚未被接受')
                self._cancel_completed = True
        else:
            self.get_logger().warn('无法取消目标：目标句柄不存在')
            self._cancel_completed = True
    
    def cancel_done(self, future):
        self.get_logger().info('收到取消响应')
        cancel_response = future.result()
        
        # 详细记录取消响应信息
        self.get_logger().info(f'取消响应内容: {cancel_response}')
        self.get_logger().info(f'取消响应返回码: {cancel_response.return_code}')
        
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('目标成功取消')
            for goal in cancel_response.goals_canceling:
                self.get_logger().info(f'取消的目标ID: {goal.goal_id.uuid}')
        else:
            self.get_logger().info('目标取消失败或已经完成')
        
        self._cancel_completed = True

def main():
    rclpy.init()
    action_client = FibonacciActionClient()
    
    # 获取命令行参数
    order = 10  # 默认值
    cancel_after = None  # 默认不取消
    
    # 处理命令行参数
    if len(sys.argv) > 1:
        try:
            order = int(sys.argv[1])
            action_client.get_logger().info(f'设置计算斐波那契数列长度为: {order}')
        except ValueError:
            action_client.get_logger().error('第一个参数必须是整数')
            action_client.get_logger().error(f'使用默认值: {order}')
    
    # 检查是否需要取消
    if len(sys.argv) > 2:
        try:
            cancel_after = float(sys.argv[2])
            action_client.get_logger().info(f'将在 {cancel_after} 秒后取消目标')
        except ValueError:
            if sys.argv[2].lower() == 'cancel':
                cancel_after = 3.0  # 默认取消时间
                action_client.get_logger().info(f'将在 {cancel_after} 秒后取消目标')
            else:
                action_client.get_logger().error('第二个参数必须是数字或"cancel"')
                action_client.get_logger().error('不会取消目标')
    
    # 发送目标
    action_client.send_goal(order)
    
    # 如果需要取消，启动取消线程
    if cancel_after is not None:
        def cancel_after_delay():
            action_client.get_logger().info(f'将在 {cancel_after} 秒后取消目标')
            time.sleep(cancel_after)
            action_client.get_logger().info('开始取消目标')
            action_client.cancel_goal()
        
        threading.Thread(target=cancel_after_delay, daemon=True).start()
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()