#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ComputeRectangleArea

class ComputeAreaServerNode(Node):
    def __init__(self):
        super().__init__("compute_area_node")
        self.server = self.create_service(ComputeRectangleArea, "compute_area", self.compute_area_callback)
        self.get_logger().info("Compute area server has been started")

    def compute_area_callback(self, request, response):
        response.area = request.lenght * request.width
        self.get_logger().info(f"Hesaplama işlemi yapildi. Result: {response.area}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ComputeAreaServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

