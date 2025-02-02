#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SumTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("sum_two_int_server")
        self.server = self.create_service(AddTwoInts, "sum_two_ints", self.sum_two_ints_callback)
        self.get_logger().info("Sum two ints server has been started")

    def sum_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"İşlem: {request.a} + {request.b} = {response.sum}")
        return response   


def main(args=None):
    rclpy.init(args=args)
    node = SumTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()    