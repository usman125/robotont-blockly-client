#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.publisher_ = self.create_publisher(Int64, "number_counter", 10)
        self.subsciber_ = self.create_subscription(
            Int64, "number", self.callback_number, 10)
        self.get_logger().info("Number counter node started now")

    def callback_number(self, counter: Int64):
        self.get_logger().info("data: " + str(counter.data))
        self.publisher_.publish(counter)
        

def main(args=None):
    rclpy.init(args=args) 
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()