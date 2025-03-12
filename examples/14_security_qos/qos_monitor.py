#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos_event import QoSLivelinessLostInfo, QoSDeadlineMissedInfo
from rclpy.qos_event import QoSRequestedDeadlineMissedInfo, QoSLivelinessChangedInfo
from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
import threading
import time
import datetime
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

class QoSMonitor(Node):
    """
    QoS监控工具，用于监视和分析ROS2系统中的QoS设置和性能
    """
    def __init__(self):
        super().__init__('qos_monitor')
        
        # 存储QoS事件和性能数据
        self.deadline_misses = []
        self.liveliness_changes = []
        self.message_latencies = {}
        self.message_counts = {}
        self.topic_qos_settings = {}
        
        # 创建QoS事件回调
        pub_callbacks = PublisherEventCallbacks(
            deadline=self.on_publisher_deadline_missed,
            liveliness=self.on_publisher_liveliness_lost
        )
        
        sub_callbacks = SubscriptionEventCallbacks(
            deadline=self.on_subscription_deadline_missed,
            liveliness=self.on_subscription_liveliness_changed
        )
        
        # 创建测试发布者（带有严格的截止时间）
        deadline_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            deadline=rclpy.duration.Duration(seconds=0.5)  # 500ms截止时间
        )
        
        self.deadline_publisher = self.create_publisher(
            String, 'qos_monitor/deadline_test', 
            deadline_qos, event_callbacks=pub_callbacks)
        
        # 创建测试发布者（带有活跃度设置）
        liveliness_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC,
            liveliness_lease_duration=rclpy.duration.Duration(seconds=1.0)  # 1秒租约
        )
        
        self.liveliness_publisher = self.create_publisher(
            String, 'qos_monitor/liveliness_test', 
            liveliness_qos, event_callbacks=pub_callbacks)
        
        # 创建相应的订阅者
        self.deadline_subscriber = self.create_subscription(
            String, 'qos_monitor/deadline_test',
            self.deadline_callback, deadline_qos,
            event_callbacks=sub_callbacks)
        
        self.liveliness_subscriber = self.create_subscription(
            String, 'qos_monitor/liveliness_test',
            self.liveliness_callback, liveliness_qos,
            event_callbacks=sub_callbacks)
        
        # 创建QoS发现服务
        self.create_timer(5.0, self.discover_topics)
        
        # 创建性能监控定时器
        self.create_timer(1.0, self.report_performance)
        
        # 创建发布线程
        self.stop_flag = False
        self.publish_thread = threading.Thread(target=self.publish_test_messages)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        # 创建可视化线程
        self.viz_thread = threading.Thread(target=self.visualize_data)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info('QoS监控工具已启动')
    
    def on_publisher_deadline_missed(self, event: QoSDeadlineMissedInfo):
        """
        发布者截止时间未达成事件回调
        """
        now = datetime.datetime.now()
        self.deadline_misses.append((now, 'publisher', event.total_count))
        self.get_logger().warning(f'发布者截止时间未达成! 总计: {event.total_count}')
    
    def on_publisher_liveliness_lost(self, event: QoSLivelinessLostInfo):
        """
        发布者活跃度丢失事件回调
        """
        now = datetime.datetime.now()
        self.liveliness_changes.append((now, 'publisher_lost', event.total_count))
        self.get_logger().warning(f'发布者活跃度丢失! 总计: {event.total_count}')
    
    def on_subscription_deadline_missed(self, event: QoSRequestedDeadlineMissedInfo):
        """
        订阅者截止时间未达成事件回调
        """
        now = datetime.datetime.now()
        self.deadline_misses.append((now, 'subscriber', event.total_count))
        self.get_logger().warning(f'订阅者截止时间未达成! 总计: {event.total_count}')
    
    def on_subscription_liveliness_changed(self, event: QoSLivelinessChangedInfo):
        """
        订阅者活跃度变化事件回调
        """
        now = datetime.datetime.now()
        status = 'active' if event.alive_count > 0 else 'inactive'
        self.liveliness_changes.append((now, f'subscriber_{status}', event.alive_count))
        self.get_logger().info(
            f'活跃度变化: 活跃={event.alive_count}, 非活跃={event.not_alive_count}, '
            f'总计变化={event.alive_count_change}')
    
    def deadline_callback(self, msg):
        """
        截止时间测试主题的回调函数
        """
        # 计算消息延迟
        try:
            send_time = float(msg.data.split('time=')[1].split(',')[0])
            receive_time = time.time()
            latency = receive_time - send_time
            
            if 'deadline_test' not in self.message_latencies:
                self.message_latencies['deadline_test'] = []
            
            self.message_latencies['deadline_test'].append((receive_time, latency))
            
            # 更新消息计数
            if 'deadline_test' not in self.message_counts:
                self.message_counts['deadline_test'] = 0
            self.message_counts['deadline_test'] += 1
            
            self.get_logger().info(f'收到截止时间测试消息: {msg.data}, 延迟: {latency:.6f}秒')
        except Exception as e:
            self.get_logger().error(f'处理截止时间消息时出错: {str(e)}')
    
    def liveliness_callback(self, msg):
        """
        活跃度测试主题的回调函数
        """
        # 计算消息延迟
        try:
            send_time = float(msg.data.split('time=')[1].split(',')[0])
            receive_time = time.time()
            latency = receive_time - send_time
            
            if 'liveliness_test' not in self.message_latencies:
                self.message_latencies['liveliness_test'] = []
            
            self.message_latencies['liveliness_test'].append((receive_time, latency))
            
            # 更新消息计数
            if 'liveliness_test' not in self.message_counts:
                self.message_counts['liveliness_test'] = 0
            self.message_counts['liveliness_test'] += 1
            
            self.get_logger().info(f'收到活跃度测试消息: {msg.data}, 延迟: {latency:.6f}秒')
        except Exception as e:
            self.get_logger().error(f'处理活跃度消息时出错: {str(e)}')
    
    def publish_test_messages(self):
        """
        发布测试消息
        """
        count = 0
        normal_interval = 0.1  # 正常发布间隔 (100ms)
        missed_interval = 0.6  # 错过截止时间的间隔 (600ms)
        
        while not self.stop_flag:
            count += 1
            current_time = time.time()
            
            # 创建消息
            deadline_msg = String()
            deadline_msg.data = f'截止时间测试 #{count}, time={current_time}, 发送于 {datetime.datetime.now()}'
            
            liveliness_msg = String()
            liveliness_msg.data = f'活跃度测试 #{count}, time={current_time}, 发送于 {datetime.datetime.now()}'
            
            # 发布消息
            self.deadline_publisher.publish(deadline_msg)
            self.liveliness_publisher.publish(liveliness_msg)
            
            # 断言活跃度（每5个消息）
            if count % 5 == 0:
                self.liveliness_publisher.assert_liveliness()
            
            # 每10个消息故意错过一次截止时间
            if count % 10 == 0:
                self.get_logger().warning(f'故意延迟发布以错过截止时间...')
                time.sleep(missed_interval)
            else:
                time.sleep(normal_interval)
    
    def discover_topics(self):
        """
        发现系统中的主题和它们的QoS设置
        """
        # 在实际应用中，这里应该使用ROS2的内省API来获取主题的QoS设置
        # 由于这需要更复杂的实现，这里我们只记录我们自己创建的主题
        
        self.topic_qos_settings['qos_monitor/deadline_test'] = {
            'reliability': 'RELIABLE',
            'durability': 'VOLATILE',
            'history': 'KEEP_LAST (10)',
            'deadline': '500ms'
        }
        
        self.topic_qos_settings['qos_monitor/liveliness_test'] = {
            'reliability': 'RELIABLE',
            'durability': 'VOLATILE',
            'history': 'KEEP_LAST (10)',
            'liveliness': 'MANUAL_BY_TOPIC',
            'lease_duration': '1000ms'
        }
        
        # 打印发现的主题
        self.get_logger().info('发现的主题和QoS设置:')
        for topic, settings in self.topic_qos_settings.items():
            self.get_logger().info(f'  {topic}:')
            for key, value in settings.items():
                self.get_logger().info(f'    {key}: {value}')
    
    def report_performance(self):
        """
        报告QoS性能指标
        """
        now = time.time()
        
        # 计算消息速率
        rates = {}
        for topic, latencies in self.message_latencies.items():
            # 只考虑最近5秒的消息
            recent_msgs = [l for t, l in latencies if now - t < 5.0]
            
            if recent_msgs:
                rates[topic] = len(recent_msgs) / 5.0  # 消息/秒
                avg_latency = sum(recent_msgs) / len(recent_msgs)
                max_latency = max(recent_msgs)
                min_latency = min(recent_msgs)
                
                self.get_logger().info(
                    f'主题 {topic} 性能: 速率={rates[topic]:.2f}消息/秒, '
                    f'平均延迟={avg_latency*1000:.2f}ms, '
                    f'最大={max_latency*1000:.2f}ms, '
                    f'最小={min_latency*1000:.2f}ms')
        
        # 报告QoS事件
        recent_deadline_misses = [d for d, _, _ in self.deadline_misses 
                               if (now - d.timestamp()) < 5.0]
        if recent_deadline_misses:
            self.get_logger().warning(f'最近5秒内有 {len(recent_deadline_misses)} 次截止时间未达成')
        
        recent_liveliness_changes = [l for l, _, _ in self.liveliness_changes 
                                  if (now - l.timestamp()) < 5.0]
        if recent_liveliness_changes:
            self.get_logger().info(f'最近5秒内有 {len(recent_liveliness_changes)} 次活跃度变化')
    
    def visualize_data(self):
        """
        可视化QoS性能数据
        """
        try:
            # 创建图表
            plt.ion()  # 开启交互模式
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
            fig.canvas.manager.set_window_title('ROS2 QoS监控器')
            
            # 初始化空数据
            times1 = []
            latencies1 = []
            times2 = []
            latencies2 = []
            
            # 创建线图
            line1, = ax1.plot(times1, latencies1, 'b-', label='截止时间测试')
            line2, = ax2.plot(times2, latencies2, 'r-', label='活跃度测试')
            
            # 设置图表属性
            ax1.set_title('消息延迟 - 截止时间测试')
            ax1.set_xlabel('时间 (s)')
            ax1.set_ylabel('延迟 (ms)')
            ax1.grid(True)
            
            ax2.set_title('消息延迟 - 活跃度测试')
            ax2.set_xlabel('时间 (s)')
            ax2.set_ylabel('延迟 (ms)')
            ax2.grid(True)
            
            # 添加图例
            ax1.legend()
            ax2.legend()
            
            # 紧凑布局
            plt.tight_layout()
            
            # 显示图表
            plt.show()
            
            start_time = time.time()
            
            # 更新循环
            while not self.stop_flag:
                # 更新数据
                if 'deadline_test' in self.message_latencies:
                    # 只保留最近30秒的数据
                    deadline_data = [(t - start_time, l * 1000) for t, l in self.message_latencies['deadline_test'] 
                                    if time.time() - t < 30.0]
                    if deadline_data:
                        times1, latencies1 = zip(*deadline_data)
                        line1.set_data(times1, latencies1)
                        ax1.relim()
                        ax1.autoscale_view()
                
                if 'liveliness_test' in self.message_latencies:
                    # 只保留最近30秒的数据
                    liveliness_data = [(t - start_time, l * 1000) for t, l in self.message_latencies['liveliness_test'] 
                                      if time.time() - t < 30.0]
                    if liveliness_data:
                        times2, latencies2 = zip(*liveliness_data)
                        line2.set_data(times2, latencies2)
                        ax2.relim()
                        ax2.autoscale_view()
                
                # 重绘图表
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                
                # 暂停一下
                time.sleep(0.5)
                
        except Exception as e:
            self.get_logger().error(f'可视化数据时出错: {str(e)}')
    
    def __del__(self):
        self.stop_flag = True
        if hasattr(self, 'publish_thread'):
            self.publish_thread.join()
        if hasattr(self, 'viz_thread'):
            self.viz_thread.join()

def main(args=None):
    rclpy.init(args=args)
    qos_monitor = QoSMonitor()
    
    try:
        rclpy.spin(qos_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        qos_monitor.stop_flag = True
        qos_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
