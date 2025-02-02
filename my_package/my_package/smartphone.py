#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.subscriber = self.create_subscription(String, "robot_news", self.robot_news_callback,10)
        self.get_logger().info("Smartphone has been started")

    def robot_news_callback(self, msg):
        self.get_logger().info(f"Publisher'dan alinan veri: {msg.data}")

def main(args=None):
    rclpy.init()
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
