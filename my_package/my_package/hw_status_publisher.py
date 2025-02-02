#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus

class MyNode(Node):
    
    def __init__(self):
        super().__init__("hw_status_node")
        self.publisher = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self._timer = self.create_timer(0.8, self.publis_hardware_status)
        self.get_logger().info("Hardware status publisher has been started")

    def publis_hardware_status(self):
        msg = HardwareStatus()
        msg.temprature = 64
        msg.are_motors_ready = True
        msg.debug_message = "Working now!"
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()