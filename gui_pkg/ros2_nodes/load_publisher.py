import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
import random


class LoadNode(Node):
    def __init__(self):
        super().__init__("load_node")
        self.i = 0
        self.load_publisher = self.create_publisher(String, "qrqr", 10)
        self.timer_ = self.create_timer(5, self.publish_load)
        self.get_logger().info("Load Publisher has been started")

    def publish_load(self):
        msg = String()
        load_dict = ["Q1", "Q2", "Q3", "Q4", "Q5", "Q10", "Q11", "Q50", "Q51", "Q52"]
        msg.data = load_dict[self.i]
        self.i += 1
        self.load_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LoadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()