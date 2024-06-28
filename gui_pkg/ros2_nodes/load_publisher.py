import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32
import random


class LoadNode(Node):
    def __init__(self):
        super().__init__("load_node")

        self.load_publisher = self.create_publisher(Int32, "load_info", 10)
        self.timer_ = self.create_timer(30, self.publish_load)
        self.get_logger().info("Load Publisher has been started")

    def publish_load(self):
        msg = Int32()
        load_dict = [0, 25, 50]
        msg.data = random.choice(load_dict)
        self.load_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LoadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()