import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State
import time

class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__('lifecycle_node_manager')
        
        # 要管理的生命周期节点的名称
        self.node_name = 'basic_lifecycle_node'
        
        # 创建服务客户端来获取节点状态
        self.state_client = self.create_client(
            GetState,
            f'/{self.node_name}/get_state'
        )
        
        # 创建服务客户端来改变节点状态
        self.change_state_client = self.create_client(
            ChangeState,
            f'/{self.node_name}/change_state'
        )
        
        # 等待服务可用
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待状态服务可用...')
        
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待状态改变服务可用...')
        
        self.get_logger().info('生命周期节点管理器已准备就绪')
    
    def get_node_state(self):
        """获取节点当前状态"""
        req = GetState.Request()
        future = self.state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            state = future.result().current_state
            state_name = self.state_id_to_label(state.id)
            self.get_logger().info(f'当前节点状态: {state_name}')
            return state.id
        else:
            self.get_logger().error('无法获取节点状态')
            return None
    
    def change_node_state(self, transition_id):
        """改变节点状态"""
        req = ChangeState.Request()
        req.transition.id = transition_id
        
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            transition_name = self.transition_id_to_label(transition_id)
            self.get_logger().info(f'状态转换成功: {transition_name}')
            return True
        else:
            self.get_logger().error('状态转换失败')
            return False
    
    def state_id_to_label(self, state_id):
        """将状态ID转换为可读的标签"""
        states = {
            State.PRIMARY_STATE_UNCONFIGURED: '未配置',
            State.PRIMARY_STATE_INACTIVE: '非活动',
            State.PRIMARY_STATE_ACTIVE: '活动',
            State.PRIMARY_STATE_FINALIZED: '已完成',
            State.TRANSITION_STATE_CONFIGURING: '配置中',
            State.TRANSITION_STATE_CLEANINGUP: '清理中',
            State.TRANSITION_STATE_ACTIVATING: '激活中',
            State.TRANSITION_STATE_DEACTIVATING: '停用中',
            State.TRANSITION_STATE_SHUTTINGDOWN: '关闭中',
            State.PRIMARY_STATE_UNKNOWN: '未知'
        }
        return states.get(state_id, f'未知状态({state_id})')
    
    def transition_id_to_label(self, transition_id):
        """将转换ID转换为可读的标签"""
        transitions = {
            Transition.TRANSITION_CONFIGURE: '配置',
            Transition.TRANSITION_CLEANUP: '清理',
            Transition.TRANSITION_ACTIVATE: '激活',
            Transition.TRANSITION_DEACTIVATE: '停用',
            Transition.TRANSITION_UNCONFIGURED_SHUTDOWN: '未配置关闭',
            Transition.TRANSITION_INACTIVE_SHUTDOWN: '非活动关闭',
            Transition.TRANSITION_ACTIVE_SHUTDOWN: '活动关闭',
            Transition.TRANSITION_CREATE: '创建',
            Transition.TRANSITION_DESTROY: '销毁'
        }
        return transitions.get(transition_id, f'未知转换({transition_id})')
    
    def run_lifecycle_sequence(self):
        """运行一个完整的生命周期序列"""
        # 获取初始状态
        state = self.get_node_state()
        
        # 配置节点
        self.get_logger().info('尝试配置节点...')
        if self.change_node_state(Transition.TRANSITION_CONFIGURE):
            time.sleep(2.0)  # 等待一会儿
        
        # 激活节点
        self.get_logger().info('尝试激活节点...')
        if self.change_node_state(Transition.TRANSITION_ACTIVATE):
            time.sleep(5.0)  # 等待一段时间让节点运行
        
        # 停用节点
        self.get_logger().info('尝试停用节点...')
        if self.change_node_state(Transition.TRANSITION_DEACTIVATE):
            time.sleep(2.0)  # 等待一会儿
        
        # 清理节点
        self.get_logger().info('尝试清理节点...')
        if self.change_node_state(Transition.TRANSITION_CLEANUP):
            time.sleep(2.0)  # 等待一会儿
        
        # 关闭节点
        self.get_logger().info('尝试关闭节点...')
        self.change_node_state(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)

def main():
    rclpy.init()
    manager = LifecycleNodeManager()
    
    try:
        # 运行生命周期序列
        manager.run_lifecycle_sequence()
        
        # 继续运行一会儿管理器
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
