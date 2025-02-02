import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class SimpleQoSSubscriber(Node):

    def __init__(self):
        super().__init__("simple_qos_subscriber")
        
        self.qos_profile_sub = QoSProfile(depth=10)

        self.declare_parameter("reliability", "system_default")
        self.declare_parameter("durability", "system_default")

        self.reliability =  self.get_parameter("reliability").get_parameter_value().string_value
        self.durability = self.get_parameter("durability").get_parameter_value().string_value

        if self.reliability == "best_effort":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info("[Reliability]: Best Effort")
        elif self.reliability == "reliable":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info("[Reliability]: Reliable")
        elif self.reliability == "system_default":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Reliability]: System Default")
        else:
            self.get_logger().error(f"Selected relibiality QoS: {self.reliability} dosen't exists")
            return
        
        if self.durability == "volatile":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.VOLATILE
            self.get_logger().info("[Durability]: Volatile")
        elif self.durability  == "transient_local":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("[Durability]: Transient Local")
        elif self.durability == "system_default":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Durability]: System Default")
        else:
            self.get_logger().error(f"Selected durability QoS: {self.durability} dosen't exists")
            return
        
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, self.qos_profile_sub)
        self.sub_

    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)


def main():
    rclpy.init()

    simple_qos_subscriber = SimpleQoSSubscriber()
    rclpy.spin(simple_qos_subscriber)
    
    simple_qos_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()