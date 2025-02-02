#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts

class SumTwoIntsClient(Node):
    
    def __init__(self):
        super().__init__("sum_two_ints_client")
        self.call_sum_two_ints_server(6,8)

        
    def call_sum_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "sum_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server AddTwoInts...")   
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.sum_two_ints_callback,a=a,b=b))

    def sum_two_ints_callback(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"Islem: {a} + {b} = {response.sum}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")        

def main(args=None):
    rclpy.init(args=args)
    node = SumTwoIntsClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()