from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(package="dashboard_bridge", executable="dashboard_bridge_node", name="dashboard_bridge", output="screen"),
            Node(package="dashboard_bridge", executable="camera_bridge_node", name="camera_bridge", output="screen"),
            Node(package="dashboard_bridge", executable="thermal_bridge_node", name="thermal_bridge", output="screen"),
            Node(package="dashboard_bridge", executable="hardware_control_node", name="hardware_control", output="screen"),
            Node(package="dashboard_bridge", executable="flexbe_adapter_node", name="flexbe_adapter", output="screen"),
            Node(package="dashboard_bridge", executable="diagnostics_node", name="diagnostics_node", output="screen"),
            Node(package="dashboard_bridge", executable="emergency_control_node", name="emergency_control", output="screen"),
        ]
    )
