#!/usr/bin/env python3


import asyncio
import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import Log
from view_tui import ViewTUI
from datetime import datetime


NODES_TO_CAPTURE = "nodes_to_capture"
LOG_QUEUE_SIZE = "log_queue_size"
LOG_QUEUE_SIZE_DEFAULT = 100
TOPIC = "/rosout"

class Viewer(Node):
    def __init__(self):
        super().__init__("terminlog_viewer")
        self.init_parameters()
        # self.create_timer(2, self.timer_callback)
        self.create_subscription(Log, TOPIC, self.handler, qos_profile=10)

        self.app = ViewTUI(self.get_parameter(NODES_TO_CAPTURE).value)

    def handler(self, msg:Log):
        """parse rosout topic log message 

        Args:
            msg (Log): rcl_interfaces.msg
        """
        # print(msg.level)
        ros_time = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec / 1e9)
        self.app.update(ros_time,
                        msg.level,
                        msg.name,
                        msg.msg)

    def init_parameters(self):
        self.declare_parameter(NODES_TO_CAPTURE, [])
        self.declare_parameter(LOG_QUEUE_SIZE, LOG_QUEUE_SIZE_DEFAULT)
    def timer_callback(self):
        # return
        # log_stamp = self.get_clock().now()

        # Convert the ROS time to a Python datetime object
        # ROS time (log_stamp) is a Time object, and we need to convert it to a datetime
        # log_time = log_stamp.to_msg()  # Convert ROS Time to ROS message
        # ros_time = datetime.fromtimestamp(log_time.sec + log_time.nanosec / 1e9)

        
        self.get_logger().info("info msg")
        self.get_logger().warning("warn msg")
        self.get_logger().error("error msg")
        

async def main():
    rclpy.init()
    
    
    node = Viewer()

    def ros_spin():
        rclpy.spin(node)

    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, ros_spin)
    await node.app.run_async()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())