import rclpy
from rclpy.executors import MultiThreadedExecutor
from my_robot_pkg.component_examples.publisher_component import PublisherComponent
from my_robot_pkg.component_examples.subscriber_component import SubscriberComponent

def main(args=None):
    rclpy.init(args=args)
    
    # 创建组件实例
    publisher = PublisherComponent('container_publisher')
    subscriber = SubscriberComponent('container_subscriber')
    
    # 配置发布者参数
    publisher.set_parameters([
        rclpy.parameter.Parameter('publish_rate', value=2.0)  # 每秒发布2次
    ])
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    
    # 添加组件到执行器
    executor.add_node(publisher)
    executor.add_node(subscriber)
    
    print('组件容器已启动，按Ctrl+C退出')
    
    try:
        # 运行执行器
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
