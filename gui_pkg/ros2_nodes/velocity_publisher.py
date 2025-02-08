import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32
import random
import time


class VelocityNode(Node):
    def __init__(self):
        super().__init__("load_node")

        self.first_velocity = 0
        self.last_velocity = 5
        self.velocity_publisher = self.create_publisher(Int32, "velocity_info", 10)
        self.timer_ = self.create_timer(1, self.publish_velocity)
        self.get_logger().info("Velocity Publisher has been started")

    def publish_velocity(self):
        msg = Int32()
        choices = [1, 2, 3, 3, 3, 3]
        self.speed = random.choice(choices)
        if self.speed == 1:
            i = 0
            for i in range(5):
                if self.first_velocity < 6:
                    self.first_velocity += 1
                    msg.data = self.first_velocity
                    self.velocity_publisher.publish(msg)
                    i += 1
                    time.sleep(1)
                else:
                    self.first_velocity = 0
        if self.speed == 2:
            i = 5
            for i in range(5):
                if self.last_velocity > 0:
                    self.last_velocity -= 1
                    msg.data = self.last_velocity
                    self.velocity_publisher.publish(msg)
                    i -= 1
                    time.sleep(1)
                else:
                    self.last_velocity = 5
                
        if self.speed == 3:
            velocity_choices = [4, 5]
            msg.data = random.choice(velocity_choices)
            self.velocity_publisher.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()