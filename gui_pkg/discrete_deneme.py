#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
import random


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        self.robot_name_ = "Alperen"
        self.publisher_ = self.create_publisher(Int64, "robot_temprature", 10)
        self.timer_ = self.create_timer(2, self.publish_news)
        self.get_logger().info("Robot News Station has been started")
        self.counter_ = 0

    def publish_news(self):
        msg = Int64()
        self.counter_ += 1
        msg.data = self.counter_
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()