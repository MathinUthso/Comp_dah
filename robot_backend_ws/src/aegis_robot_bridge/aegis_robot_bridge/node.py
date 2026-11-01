"""ROS 2 bridge between the dashboard and the ESP32 M3508 drive firmware.

Contract (identical to the Gazebo adapter, so the dashboard needs no changes):

* subscribes ``/cmd_vel`` (``geometry_msgs/Twist``) and streams wheel RPM over USB serial
* serves ``/emergency/estop`` and ``/emergency/reset`` (``std_srvs/Trigger``)
* publishes ``/emergency/status``, ``/hardware/odometry`` and ``/diagnostics``

Safety: the last ``/cmd_vel`` is held for ``command_hold_s`` and then zeroed, so
a dropped Wi-Fi link or a closed browser tab stops the robot. Commands are also
zeroed when telemetry goes stale, and the firmware zeroes the motors on its own
if the serial stream stops.
"""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from aegis_robot_bridge.protocol import (
    Ack,
    Telemetry,
    WheelOdometry,
    format_estop,
    format_reset,
    format_stop,
    format_velocity,
    parse_line,
    rpm_to_mps,
    twist_to_wheel_rpm,
    yaw_to_quaternion,
)
from aegis_robot_bridge.serial_link import SerialLink


class RobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("aegis_robot_bridge")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("track_width", 0.28)
        self.declare_parameter("ticks_per_wheel_rev", 8192.0)
        self.declare_parameter("max_wheel_rpm", 1500.0)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("command_hold_s", 0.35)
        self.declare_parameter("telemetry_timeout_s", 1.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.max_wheel_rpm = float(self.get_parameter("max_wheel_rpm").value)
        self.command_hold_s = max(0.05, float(self.get_parameter("command_hold_s").value))
        self.telemetry_timeout_s = max(0.2, float(self.get_parameter("telemetry_timeout_s").value))
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)

        self.odometry = WheelOdometry(
            self.wheel_radius,
            self.track_width,
            float(self.get_parameter("ticks_per_wheel_rev").value),
        )
        self.link = SerialLink(port=port, baud=baud)

        self._estop_latched = False
        self._cmd_left_rpm = 0.0
        self._cmd_right_rpm = 0.0
        self._last_cmd_monotonic = 0.0
        self._last_telemetry: Telemetry | None = None
        self._last_telemetry_monotonic = 0.0
        self._ack_event = False
        self._ack: Ack | None = None

        self._odom_pub = self.create_publisher(Odometry, "/hardware/odometry", 10)
        self._emergency_pub = self.create_publisher(Bool, "/emergency/status", 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_service(Trigger, "/emergency/estop", self._on_estop)
        self.create_service(Trigger, "/emergency/reset", self._on_reset)

        self.link.start()
        self.create_timer(1.0 / rate_hz, self._tick_command)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"ESP32 drive bridge starting: port={self.link.port or 'auto'} baud={baud} "
            f"r={self.wheel_radius} m track={self.track_width} m"
        )

    # ------------------------------------------------------------------
    # Drive path
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        left, right = twist_to_wheel_rpm(
            float(msg.linear.x),
            float(msg.angular.z),
            self.wheel_radius,
            self.track_width,
            self.max_wheel_rpm,
        )
        self._cmd_left_rpm = left
        self._cmd_right_rpm = right
        self._last_cmd_monotonic = time.monotonic()

    def _link_is_healthy(self) -> bool:
        if not self.link.connected:
            return False
        return (time.monotonic() - self._last_telemetry_monotonic) <= self.telemetry_timeout_s

    def _tick_command(self) -> None:
        self._process_serial_lines()
        now = time.monotonic()

        if self._estop_latched:
            left = right = 0.0
        elif not self._link_is_healthy() or (now - self._last_cmd_monotonic) > self.command_hold_s:
            # Stale link or stale /cmd_vel: hold zero rather than the last command.
            left = right = 0.0
        else:
            left, right = self._cmd_left_rpm, self._cmd_right_rpm

        if self.link.connected:
            self.link.send(format_velocity(left, right, self.max_wheel_rpm))

    # ------------------------------------------------------------------
    # Serial telemetry
    # ------------------------------------------------------------------
    def _process_serial_lines(self) -> None:
        for line in self.link.drain():
            parsed = parse_line(line)
            if isinstance(parsed, Ack):
                self._ack = parsed
                self._ack_event = True
            elif isinstance(parsed, Telemetry):
                self._on_telemetry(parsed)
            elif line.startswith("#"):
                self.get_logger().debug(f"esp32: {line}")

    def _on_telemetry(self, telemetry: Telemetry) -> None:
        self._last_telemetry = telemetry
        self._last_telemetry_monotonic = time.monotonic()
        if telemetry.estop and not self._estop_latched:
            self.get_logger().warning("ESP32 reported a latched E-stop")
            self._estop_latched = True

        pose = self.odometry.update(telemetry.ticks_left, telemetry.ticks_right)

        left_mps = rpm_to_mps(telemetry.rpm_left, self.wheel_radius)
        right_mps = rpm_to_mps(telemetry.rpm_right, self.wheel_radius)

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = (left_mps + right_mps) / 2.0
        message.twist.twist.angular.z = (right_mps - left_mps) / self.track_width
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.05
        message.twist.covariance[0] = 0.02
        message.twist.covariance[35] = 0.05
        self._odom_pub.publish(message)

    def _send_and_wait(self, line: str, timeout_s: float = 0.4) -> tuple[bool, str]:
        """Send an acknowledged command and wait briefly for the ESP32 reply."""
        if not self.link.connected:
            return False, "serial link down"
        self._ack = None
        self._ack_event = False
        self.link.send(line)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._process_serial_lines()
            if self._ack_event:
                ack = self._ack
                if ack is not None:
                    return ack.ok, ack.message
                break
            time.sleep(0.01)
        return False, f"no acknowledgement for {line.split(',')[0]}"

    # ------------------------------------------------------------------
    # Emergency services
    # ------------------------------------------------------------------
    def _on_estop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._estop_latched = True
        self._cmd_left_rpm = 0.0
        self._cmd_right_rpm = 0.0
        self._last_cmd_monotonic = 0.0

        acked, detail = self._send_and_wait(format_estop())
        self.link.send(format_stop())
        self._publish_emergency_status()
        response.success = acked
        response.message = (
            "E-stop latched on the ESP32; drive output cut"
            if acked
            else f"E-stop latched locally but NOT confirmed by the ESP32: {detail}"
        )
        self.get_logger().warning(response.message)
        return response

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        acked, detail = self._send_and_wait(format_reset())
        if acked:
            self._estop_latched = False
            self._last_cmd_monotonic = 0.0
        self._publish_emergency_status()
        response.success = acked
        response.message = (
            "E-stop cleared; bridge idle until the next /cmd_vel"
            if acked
            else f"E-stop still latched locally: {detail}"
        )
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Status / diagnostics
    # ------------------------------------------------------------------
    def _publish_emergency_status(self) -> None:
        self._emergency_pub.publish(Bool(data=self._estop_latched))

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        age = now - self._last_telemetry_monotonic if self._last_telemetry_monotonic else float("inf")
        link_ok = self._link_is_healthy()

        link_status = DiagnosticStatus()
        link_status.name = "ESP32 Link"
        link_status.hardware_id = self.link.port or "serial"
        link_status.level = DiagnosticStatus.OK if link_ok else DiagnosticStatus.ERROR
        if link_ok:
            link_status.message = f"{self.link.port} @ {self.link.baud} baud, telemetry {age:.2f} s old"
        elif not self.link.connected:
            link_status.message = f"serial link down ({self.link.last_error or 'no device'})"
        else:
            link_status.message = f"no telemetry for {age:.1f} s"
        link_status.values = [
            KeyValue(key="port", value=self.link.port or "auto"),
            KeyValue(key="connected", value=str(self.link.connected)),
            KeyValue(key="telemetry_age_s", value=f"{age:.2f}" if age != float("inf") else "never"),
            KeyValue(key="tx_lines", value=str(self.link.tx_count)),
            KeyValue(key="rx_lines", value=str(self.link.rx_count)),
        ]

        telemetry = self._last_telemetry
        drive_status = DiagnosticStatus()
        drive_status.name = "Drive"
        drive_status.hardware_id = "esp32_m3508"
        drive_status.level = DiagnosticStatus.ERROR if self._estop_latched else DiagnosticStatus.OK
        if self._estop_latched:
            drive_status.message = "E-STOP latched"
        elif telemetry is None:
            drive_status.message = "no motor telemetry yet"
        else:
            drive_status.message = "velocity control"
        drive_status.values = [
            KeyValue(key="estop", value=str(self._estop_latched)),
            KeyValue(key="rpm_left", value=str(telemetry.rpm_left if telemetry else 0)),
            KeyValue(key="rpm_right", value=str(telemetry.rpm_right if telemetry else 0)),
            KeyValue(key="cmd_left_rpm", value=str(telemetry.command_left_rpm if telemetry else 0)),
            KeyValue(key="cmd_right_rpm", value=str(telemetry.command_right_rpm if telemetry else 0)),
            KeyValue(key="ticks_left", value=str(telemetry.ticks_left if telemetry else 0)),
            KeyValue(key="ticks_right", value=str(telemetry.ticks_right if telemetry else 0)),
        ]

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [link_status, drive_status]
        self._diagnostics_pub.publish(message)
        self._publish_emergency_status()

    def destroy_node(self) -> bool:
        self.link.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
