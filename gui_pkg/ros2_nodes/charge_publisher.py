import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32
import random


class ChargeNode(Node):
    def __init__(self):
        super().__init__("charge_node")

        self.current_publisher = self.create_publisher(Int32, "charge_info", 10)
        self.timer_ = self.create_timer(15, self.charge_temperature)
        self.get_logger().info("Charge Publisher has been started")
        
        self.charge_level = 100

    def charge_temperature(self):
        if self.charge_level > 0:
            self.charge_level -= 1

        msg = Int32()
        msg.data = self.charge_level
        self.current_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChargeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()