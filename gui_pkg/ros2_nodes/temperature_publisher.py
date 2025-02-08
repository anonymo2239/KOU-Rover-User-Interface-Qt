import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32
import random


class TemperatureNode(Node):
    def __init__(self):
        super().__init__("temperature_node")

        self.temperature_publisher = self.create_publisher(Int32, "temperature_info", 10)
        self.timer_ = self.create_timer(10, self.publish_temperature)
        self.get_logger().info("Temperature Publisher has been started")

    def publish_temperature(self):
        msg = Int32()
        msg.data = random.randint(25, 28)
        self.temperature_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()