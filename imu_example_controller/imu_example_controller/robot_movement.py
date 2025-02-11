#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.diff_drvie_control_pub = self.create_publisher(TwistStamped, "/diff_drive_robot_controller/cmd_vel", 10)
        
        self.odom_sub = self.create_subscription(Odometry, "/diff_drive_robot_controller/odom", self.odom_callback, 10)

        self.speed = 1.0
        self.distance_to_travel = 2.0
        self.current_distance = 0.0
        self.start_position = None
        self.is_moving_forward = True

    def odom_callback(self, msg):
        if self.start_position is None:
            self.start_position = msg.pose.pose.position
            return
        
        current_position = msg.pose.pose.position
        self.current_distance = abs(current_position.x - self.start_position.x)

        if self.is_moving_forward and self.current_distance >= self.distance_to_travel:
            self.get_logger().info(f"Hedef mesafeye ulaşıldı: {self.current_distance} metre")
            self.stop_robot()
            time.sleep(1.0)
            self.is_moving_forward = False
        elif self.is_moving_forward:
            self.move_forward()
        elif not self.is_moving_forward and self.current_distance <= 0.1:
            self.get_logger().info("Başlangıç konumuna dönüldü.")
            self.stop_robot()
            rclpy.shutdown()
        elif not self.is_moving_forward:
            self.move_backward()
    
    def move_forward(self):
        msg = TwistStamped()
        msg.twist.linear.x = self.speed
        msg.twist.angular.z = 0.0
        self.diff_drvie_control_pub.publish(msg)
        self.get_logger().info("Araç ileri hareket ediyor...")

    def move_backward(self):
        msg = TwistStamped()
        msg.twist.linear.x = -self.speed
        msg.twist.angular.z = 0.0
        self.diff_drvie_control_pub.publish(msg)
        self.get_logger().info("Araç geri hareket ediyor.")

    def stop_robot(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.diff_drvie_control_pub.publish(msg)
        self.get_logger().info("Araç durdu.")


def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()