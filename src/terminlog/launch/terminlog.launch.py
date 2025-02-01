import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    return LaunchDescription([
        # Node(
        #     package='terminlog',
        #     executable='viewer.py',
        #     parameters=[],
        #     output='screen',
        #     additional_env={
        #         "PYTHONUNBUFFERED": "1"
        #     }
        # )
        ExecuteProcess(
            cmd=['gnome-terminal', '--','ros2', "run", "terminlog", "viewer.py"],
            output='screen',
            shell=False 
        )
    ])
