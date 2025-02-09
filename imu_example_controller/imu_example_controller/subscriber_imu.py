#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__("imu_subscriber")

        self.declare_parameter("imu_name", "none")

        if self.get_parameter("imu_name").get_parameter_value().string_value == "none":
            self.get_logger().error("Lütfen abone olunacak imu'nun isimini parametre olarak geçiniz!")
            rclpy.shutdown()
            return

        self.imu_name = self.get_parameter("imu_name").get_parameter_value().string_value
        self.imu_sub = self.create_subscription(Imu, self.imu_name, self.imu_sub_callback, 10)

    def imu_sub_callback(self, msg):
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        self.get_logger().info(
            f"\nAngular Velocity (z): {angular_velocity.z:.3f}\n"
            f"Linear Acceleration (x): {linear_acceleration.x:.3f}"
        )



def main():
    rclpy.init()
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == "__main__":
    main()