#!/usr/bin/env python3


import asyncio
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import Log, ParameterDescriptor
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
        #TODO: qos or depth, 
        self.create_subscription(Log, TOPIC, self.handler, qos_profile=10)
        self.nodes_name = self.get_parameter(NODES_TO_CAPTURE).get_parameter_value().string_array_value
        self.app = ViewTUI(self.nodes_name, active_node_names_cb=self.active_nodes_filter)

    def active_nodes_filter(self):
        node_names = self.get_node_names()
        return node_names

    def handler(self, msg:Log):
        """parse rosout topic log message 

        Args:
            msg (Log): rcl_interfaces.msg
        """
        if msg.name not in self.nodes_name:
            # filter un declared node
            return
        ros_time = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec / 1e9)
        self.app.update(ros_time,
                        msg.level,
                        msg.name,
                        msg.msg,
                        msg.file,
                        msg.line)

    def init_parameters(self):
        descriptor = ParameterDescriptor(
            description="Nodes to capture",
            type=ParameterType.PARAMETER_STRING_ARRAY
        )
        self.declare_parameter(NODES_TO_CAPTURE, [''], descriptor=descriptor)
        self.declare_parameter(LOG_QUEUE_SIZE, LOG_QUEUE_SIZE_DEFAULT)
        

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