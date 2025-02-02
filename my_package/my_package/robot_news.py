#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotStationNode(Node):
    
    def __init__(self):
        super().__init__("robot_news")
        self.publisher = self.create_publisher(String, "robot_news", 10)
        self._timer = self.create_timer(0.8, self.publish_news)
        self.get_logger().info("Robot news station has been started")        
        
    def publish_news(self):
        msg = String()
        msg.data = "Robot Haberlerinden Merhaba!"
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()