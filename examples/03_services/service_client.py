import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    addition_client = AdditionClient()

    if len(sys.argv) != 3:
        addition_client.get_logger().info('usage: ros2 run my_robot_pkg service_client arg1 arg2')
        addition_client.get_logger().info('example: ros2 run my_robot_pkg service_client 2 3')
        a = 2
        b = 3
    else:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    response = addition_client.send_request(a, b)
    addition_client.get_logger().info('Incoming response\nadd_two_ints_response\nsum: %d' % (response.sum))
    addition_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()