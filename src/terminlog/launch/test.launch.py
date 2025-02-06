import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="terminlog",
            executable="test_node.py",
            name="test1"
        ),
        launch_ros.actions.Node(
           package="terminlog",
            executable="test_node.py",
            name="test2",
            namespace="sss"
        )
    ])
