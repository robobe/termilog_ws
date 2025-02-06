#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        node_name="test_node"
        super().__init__(node_name)
        self.get_logger().info("Hello ROS2")
        self.t = self.create_timer(1/10, self.test_log_message)

    def test_log_message(self):
        self.get_logger().debug("This is a test log message.")
        self.get_logger().info("This is a test log message.")
        self.get_logger().warning("This is a test log message.")
        self.get_logger().error("This is a test log message.")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()