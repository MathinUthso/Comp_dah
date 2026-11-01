from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(package="camera_bridge", executable="camera_bridge_node", name="camera_bridge", output="screen"),
        ]
    )