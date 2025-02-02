#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareSubscriberNode(Node):
    def __init__(self):
        super().__init__("hardware_subscriber")
        self.subscriber = self.create_subscription(HardwareStatus, "hardware_status", self.hardware_status_callback,10)
        self.get_logger().info("Hardware Subscriber has been started")

    def hardware_status_callback(self, msg):
        self.get_logger().info(f"Temprature: {msg.temprature}, Motors Ready : {msg.are_motors_ready}, Debug Message: {msg.debug_message}")

def main(args=None):
    rclpy.init()
    node = HardwareSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()