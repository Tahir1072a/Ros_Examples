#!/usr/bin/env python3
import rclpy
import subprocess

from rclpy.node import Node
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__("imu_subscriber")

        self.file_path = "imu_report.csv"
        subprocess.run(["code", self.file_path])

        self.imu_file = open("imu_report.csv", "w")
        self.imu_file.write(f"{'time':<16} {'pose_x':<16} {'pose_y':<16} {'pose_z':<16} {'ax1':<16} {'ay1':<16} {'az1':<16} {'gx1':<16} {'gy1':<16} {'gz1':<16} {'ax2':<16} {'ay2':<16} {'az2':<16} {'gx2':<16} {'gy2':<16} {'gz2':<16} {'ax3':<16} {'ay3':<16} {'az3':<16} {'gx3':<16} {'gy3':<16} {'gz3':<16}\n")

        self.buffer = {}
    
        self.imu1_sub = self.create_subscription(Imu, "/imu1", lambda msg: self.imu_sub_callback(msg, "imu1"), 10)
        self.imu2_sub = self.create_subscription(Imu, "/imu2",  lambda msg: self.imu_sub_callback(msg, "imu2"), 10)
        self.imu3_sub = self.create_subscription(Imu, "/imu3",  lambda msg: self.imu_sub_callback(msg, "imu3"), 10)

        self.joint_state_sub = self.create_subscription(Odometry, "/diff_drive_robot_controller/odom", self.deneme, 10)

    def deneme(self, msg):
        current_position = msg.pose.pose.position
        timestamp = msg.header.stamp.sec

        if timestamp not in self.buffer:
            self.buffer[timestamp] = {}
        
        self.buffer[timestamp]["pose"] = {
            "pose_x": current_position.x,
            "pose_y": current_position.y,
            "pose_z": current_position.z
        }
        

    def imu_sub_callback(self, msg, imu_id):
        timestamp = msg.header.stamp.sec

        if timestamp not in self.buffer:
            self.buffer[timestamp] = {}

        self.buffer[timestamp][imu_id] = {
            "ax": msg.linear_acceleration.x,
            "ay": msg.linear_acceleration.y,
            "az": msg.linear_acceleration.z,
            "gx": msg.angular_velocity.x,
            "gy": msg.angular_velocity.y,
            "gz": msg.angular_velocity.z,
        }

        #self.get_logger().info(f"------------------------.debug -- x: {timestamp} imu_id: {imu_id} buffer: {self.buffer[timestamp][imu_id]} linear_acc: {msg.linear_acceleration.x}")

        if len(self.buffer[timestamp]) == 4:
            self.write_to_file(timestamp)
            del self.buffer[timestamp]

    def write_to_file(self, timestamp):
        data = self.buffer[timestamp]

        data_str = f"{timestamp:<15} "
        for imu_id in ["pose", "imu1", "imu2", "imu3"]:
            if imu_id == "pose":
                imu_data = data[imu_id]
                pose_x = float(imu_data["pose_x"])
                pose_y = float(imu_data["pose_y"])
                pose_z = float(imu_data["pose_z"])
                data_str += f"{pose_x:<15.12f}, {pose_y:<15.12f}, {pose_z:<15.12f}, "
            else:
                imu_data = data[imu_id]
                data_str += f"{imu_data['ax']:<15.12f}, {imu_data['ay']:<15.12f}, {imu_data['az']:<15.12f}, {imu_data['gx']:<15.12f}, {imu_data['gy']:<15.12f}, {imu_data['gz']:<15.12f}, "

        data_str = data_str.rstrip(", ") + "\n" 
        self.imu_file.write(data_str)
        self.imu_file.flush()

def main():
    rclpy.init()
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == "__main__":
    main()