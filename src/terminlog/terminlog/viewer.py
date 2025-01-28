#!/usr/bin/env python3


import asyncio
import rclpy
from rclpy.node import Node 
from rclpy.qos import qos_profile_system_default
from rcl_interfaces.msg import Log
from view_tui import ViewTUI
from datetime import datetime
class Viewer(Node):
    def __init__(self, app: ViewTUI):
        super().__init__("terminlog_viewer")
        self.create_timer(2, self.timer_callback)
        self.app = app
        self.app.register_filter_name("0", "name")
        self.app.register_filter_name("1", "xxx")

    def timer_callback(self):
        # return
        log_stamp = self.get_clock().now()

        # Convert the ROS time to a Python datetime object
        # ROS time (log_stamp) is a Time object, and we need to convert it to a datetime
        log_time = log_stamp.to_msg()  # Convert ROS Time to ROS message
        ros_time = datetime.fromtimestamp(log_time.sec + log_time.nanosec / 1e9)

        
        self.app.update(ros_time, Log.DEBUG, "name", "debug")
        self.app.update(ros_time, Log.INFO, "name", "info")
        self.app.update(ros_time, Log.WARN, "name", "warning")
        self.app.update(ros_time, Log.ERROR, "name", "error")
        self.app.update(ros_time, Log.INFO, "xxx", "info")
        self.app.update(ros_time, Log.WARN, "xxx", "warning")
        

async def main():
    rclpy.init()
    
    app = ViewTUI()
    node = Viewer(app)

    def ros_spin():
        rclpy.spin(node)

    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, ros_spin)
    await app.run_async()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())