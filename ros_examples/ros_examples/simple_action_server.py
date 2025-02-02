import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from bumperbot_msgs.action import Fibonacci

class SimpleActionServer(Node):
    def __init__(self):
        super().__init__("simple_action_server")

        self.action_server = ActionServer(self, Fibonacci, "fibonacci", self.goal_callback)
        self.get_logger().info("Starting the Action Server")

    def goal_callback(self, goal_handle):
        self.get_logger().info(f"Received goal request with order: {goal_handle.request.order}")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1])
            self.get_logger().info(f"Feedback: {format(feedback_msg.partial_sequence)}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
    

def main():
    rclpy.init()
    node = SimpleActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        