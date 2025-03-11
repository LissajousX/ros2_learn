import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\nadd_two_ints_request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):   
    rclpy.init(args=args)
    addition_server = AdditionServer()
    rclpy.spin(addition_server)
    addition_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()