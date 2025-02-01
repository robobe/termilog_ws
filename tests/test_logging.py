import rclpy
import pytest
import time
from rclpy.node import Node
import logging

@pytest.fixture(scope="module")
def ros2_node():
    rclpy.init()
    node = Node("test_logger_node")
    node.get_logger().set_level(logging.DEBUG)  # Enable DEBUG logs
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_log_message(ros2_node, capsys):
    ros2_node.get_logger().debug("This is a test log message.")
    ros2_node.get_logger().info("This is a test log message.")
    ros2_node.get_logger().warning("This is a test log message.")
    ros2_node.get_logger().error("This is a test log message.")

    # Give ROS some time to process logging
    time.sleep(1)

    # Capture standard output
    captured = capsys.readouterr()

    # Verify log message appears in output
    assert "This is a test log message." in captured.out
