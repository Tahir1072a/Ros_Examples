import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class SimpleQoSPublisher(Node):

    def __init__(self):
        super().__init__("simple_qos_publisher")

        self.qos_profile_pub = QoSProfile(depth=10)

        self.declare_parameter("reliability", "system_default")
        self.declare_parameter("durability", "system_default")

        self.reliability =  self.get_parameter("reliability").get_parameter_value().string_value
        self.durability = self.get_parameter("durability").get_parameter_value().string_value

        if self.reliability == "best_effort":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info("[Reliability]: Best Effort")
        elif self.reliability == "reliable":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info("[Reliability]: Reliable")
        elif self.reliability == "system_default":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Reliability]: System Default")
        else:
            self.get_logger().error(f"Selected reliability QoS: {self.reliability} dosen't exists")
            return
        
        if self.durability == "volatile":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.VOLATILE
            self.get_logger().info("[Durability]: Volatile")
        elif self.durability  == "transient_local":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("[Durability]: Transient Local")
        elif self.durability == "system_default":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Durability]: System Default")
        else:
            self.get_logger().error(f"Selected durability QoS: {self.durability} dosen't exists")
            return


        self.pub_ = self.create_publisher(String, "chatter", self.qos_profile_pub)
        self.counter_ = 0
        self.frequency_ = 1.0
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_
        self.pub_.publish(msg)
        self.counter_ += 1


def main():
    rclpy.init()

    node = SimpleQoSPublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()