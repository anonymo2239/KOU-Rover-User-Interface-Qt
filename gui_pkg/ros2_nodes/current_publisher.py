import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32
import random


class CurrentNode(Node):
    def __init__(self):
        super().__init__("current_node")

        self.current_publisher = self.create_publisher(Int32, "current_info", 10)
        self.timer_ = self.create_timer(2, self.current_temperature)
        self.get_logger().info("Current Publisher has been started")

    def current_temperature(self):
        msg = Int32()
        msg.data = random.randint(17, 25)
        self.current_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()