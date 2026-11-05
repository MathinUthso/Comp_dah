"""Bring up the Raspberry Pi drive bridge for the ESP32 M3508 robot."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    wheel_radius = LaunchConfiguration("wheel_radius")
    track_width = LaunchConfiguration("track_width")
    ticks_per_wheel_rev = LaunchConfiguration("ticks_per_wheel_rev")
    max_wheel_rpm = LaunchConfiguration("max_wheel_rpm")
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    command_hold_s = LaunchConfiguration("command_hold_s")
    max_current_a = LaunchConfiguration("max_current_a")
    position_ping_s = LaunchConfiguration("position_ping_s")
    telemetry_timeout_s = LaunchConfiguration("telemetry_timeout_s")

    bridge = Node(
        package="aegis_robot_bridge",
        executable="robot_bridge_node",
        name="aegis_robot_bridge",
        output="screen",
        parameters=[
            {
                "port": ParameterValue(port, value_type=str),
                "baud": ParameterValue(baud, value_type=int),
                "wheel_radius": ParameterValue(wheel_radius, value_type=float),
                "track_width": ParameterValue(track_width, value_type=float),
                "ticks_per_wheel_rev": ParameterValue(ticks_per_wheel_rev, value_type=float),
                "max_wheel_rpm": ParameterValue(max_wheel_rpm, value_type=float),
                "command_rate_hz": ParameterValue(command_rate_hz, value_type=float),
                "command_hold_s": ParameterValue(command_hold_s, value_type=float),
                "max_current_a": ParameterValue(max_current_a, value_type=float),
                "position_ping_s": ParameterValue(position_ping_s, value_type=float),
                "telemetry_timeout_s": ParameterValue(telemetry_timeout_s, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="", description="Serial device; empty = auto-detect the ESP32 adapter."),
            DeclareLaunchArgument("baud", default_value="115200", description="ESP32 serial baud rate."),
            DeclareLaunchArgument("wheel_radius", default_value="0.05", description="Wheel radius in metres (firmware WHEEL_RADIUS)."),
            DeclareLaunchArgument("track_width", default_value="0.28", description="Wheel separation in metres (firmware TRACK_WIDTH)."),
            DeclareLaunchArgument(
                "ticks_per_wheel_rev",
                default_value="8192.0",
                description="Encoder ticks per wheel revolution (TICKS_PER_MOTOR_REV * GEAR_RATIO).",
            ),
            DeclareLaunchArgument("max_wheel_rpm", default_value="1500.0", description="Per-wheel RPM clamp."),
            DeclareLaunchArgument("command_rate_hz", default_value="20.0", description="Serial command stream rate."),
            DeclareLaunchArgument(
                "command_hold_s",
                default_value="0.35",
                description="How long /cmd_vel is held before the bridge commands zero.",
            ),
            DeclareLaunchArgument(
                "max_current_a",
                default_value="10.0",
                description="Torque-mode current clamp in amps (firmware also clamps to 10 A).",
            ),
            DeclareLaunchArgument(
                "position_ping_s",
                default_value="0.18",
                description="Keep-alive interval while holding a position target.",
            ),
            DeclareLaunchArgument(
                "telemetry_timeout_s",
                default_value="1.0",
                description="Telemetry age after which the link is treated as unhealthy and commands are zeroed.",
            ),
            bridge,
        ]
    )
