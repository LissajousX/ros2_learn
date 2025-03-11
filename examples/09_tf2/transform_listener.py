#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.lookup_transform)  # 1Hz
        
    def lookup_transform(self):
        try:
            # 查询从base_link到camera_link的变换
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time()  # 获取最新的变换
            )
            
            # 打印变换信息
            self.get_logger().info(
                f'从{transform.header.frame_id}到{transform.child_frame_id}的变换:\n'
                f'平移: [{transform.transform.translation.x}, '
                f'{transform.transform.translation.y}, '
                f'{transform.transform.translation.z}]\n'
                f'旋转: [{transform.transform.rotation.x}, '
                f'{transform.transform.rotation.y}, '
                f'{transform.transform.rotation.z}, '
                f'{transform.transform.rotation.w}]'
            )
            
        except TransformException as ex:
            self.get_logger().error(f'无法获取变换: {ex}')

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
