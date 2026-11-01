"""Translate Gazebo Classic Aegis topics into the dashboard contract."""

from __future__ import annotations

import math
import os
import time

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan, Range, Temperature
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from aegis_sim.math_helpers import forward_cone_min_range, is_stale, next_battery_percent


class DashboardAdapter(Node):
    """Bridge the simulator's raw ROS topics to the dashboard's expected topics."""

    def __init__(self) -> None:
        super().__init__("aegis_dashboard_adapter")

        self.declare_parameter("camera_fps", 12.0)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("camera_stale_s", 1.5)
        self.declare_parameter("front_camera_topic", "/camera/front/image_raw")
        self.declare_parameter("rear_camera_topic", "/camera/rear/image_raw")
        self.declare_parameter("third_person_camera_topic", "/camera/third_person/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_sim")

        self._camera_period = 1.0 / max(1.0, float(self.get_parameter("camera_fps").value))
        self._jpeg_quality = max(1, min(100, int(self.get_parameter("jpeg_quality").value)))
        self._camera_stale_s = max(0.2, float(self.get_parameter("camera_stale_s").value))
        self._front_last_image = 0.0
        self._rear_last_image = 0.0
        self._front_last_publish = 0.0
        self._rear_last_publish = 0.0
        self._latest_scan: LaserScan | None = None
        self._estop_latched = False
        self._battery_percent = 100.0
        self._last_diagnostics = time.monotonic()

        front_topic = str(self.get_parameter("front_camera_topic").value)
        rear_topic = str(self.get_parameter("rear_camera_topic").value)
        third_person_topic = str(self.get_parameter("third_person_camera_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_in = str(self.get_parameter("cmd_vel_in").value)
        cmd_out = str(self.get_parameter("cmd_vel_out").value)

        self._front_pub = self.create_publisher(CompressedImage, "/camera/image_compressed", 10)
        self._video2_pub = self.create_publisher(CompressedImage, "/camera/video2/image_compressed", 10)
        self._webcam_pub = self.create_publisher(CompressedImage, "/camera/webcam/image_compressed", 10)
        self._video2_health_pub = self.create_publisher(Bool, "/camera/video2/healthy", 10)
        self._webcam_health_pub = self.create_publisher(Bool, "/camera/webcam/healthy", 10)
        self._primary_source_pub = self.create_publisher(String, "/camera/primary_source", 10)
        self._proximity_pub = self.create_publisher(Range, "/proximity/range", 10)
        self._thermal_pub = self.create_publisher(Temperature, "/thermal/temperature", 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._emergency_pub = self.create_publisher(Bool, "/emergency/status", 10)
        self._cmd_pub = self.create_publisher(Twist, cmd_out, 10)

        self.create_subscription(Image, front_topic, self._on_front_image, qos_profile_sensor_data)
        self.create_subscription(Image, rear_topic, self._on_rear_image, qos_profile_sensor_data)
        self.create_subscription(Image, third_person_topic, self._on_third_person_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, cmd_in, self._on_cmd_vel, 10)

        self.create_service(Trigger, "/emergency/estop", self._on_estop)
        self.create_service(Trigger, "/emergency/reset", self._on_reset)

        self.create_timer(1.0, self._publish_health)
        self.create_timer(0.1, self._publish_proximity)
        self.create_timer(1.0, self._publish_thermal)
        self.create_timer(1.0, self._publish_diagnostics)
        self.create_timer(1.0, self._publish_emergency)

        self._publish_health()
        self._publish_emergency()
        self.get_logger().info(
            "Aegis dashboard adapter ready: /emergency/estop, /emergency/reset, "
            f"{cmd_in} -> {cmd_out}"
        )

    # ------------------------------------------------------------------
    # Camera path
    # ------------------------------------------------------------------
    def _on_front_image(self, msg: Image) -> None:
        now = time.monotonic()
        self._front_last_image = now
        if now - self._front_last_publish < self._camera_period:
            return
        self._front_last_publish = now
        encoded = self._encode_image(msg)
        if encoded is None:
            return

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = "jpeg"
        compressed.data = encoded
        self._video2_pub.publish(compressed)

    def _on_third_person_image(self, msg: Image) -> None:
        now = time.monotonic()
        # Keep the main preview near the configured camera rate.
        if now - self._front_last_publish < self._camera_period:
            return
        self._front_last_publish = now
        encoded = self._encode_image(msg)
        if encoded is None:
            return

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = "jpeg"
        compressed.data = encoded
        self._front_pub.publish(compressed)

    def _on_rear_image(self, msg: Image) -> None:
        now = time.monotonic()
        self._rear_last_image = now
        if now - self._rear_last_publish < self._camera_period:
            return
        self._rear_last_publish = now
        encoded = self._encode_image(msg)
        if encoded is None:
            return

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = "jpeg"
        compressed.data = encoded
        self._webcam_pub.publish(compressed)

    def _encode_image(self, msg: Image) -> bytes | None:
        height = int(msg.height)
        width = int(msg.width)
        if height <= 0 or width <= 0:
            return None

        encoding = str(msg.encoding).lower()
        if encoding in ("rgb8", "bgr8"):
            channels = 3
        elif encoding in ("mono8", "8uc1"):
            channels = 1
        else:
            self.get_logger().warning(f"Unsupported camera encoding: {msg.encoding}", throttle_duration_sec=5.0)
            return None

        row_bytes = width * channels
        expected = height * row_bytes
        if len(msg.data) < expected:
            return None

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        step = int(msg.step) if int(msg.step) >= row_bytes else row_bytes
        if step == row_bytes:
            frame = raw[:expected].reshape((height, width, channels))
        else:
            rows = raw[: height * step].reshape((height, step))
            frame = rows[:, :row_bytes].reshape((height, width, channels))

        if channels == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            return None
        return bytes(buffer)

    # ------------------------------------------------------------------
    # Drive / safety path
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._estop_latched:
            self._publish_zero()
            return

        command = Twist()
        command.linear.x = float(msg.linear.x)
        command.linear.y = float(msg.linear.y)
        command.linear.z = float(msg.linear.z)
        command.angular.x = float(msg.angular.x)
        command.angular.y = float(msg.angular.y)
        command.angular.z = float(msg.angular.z)
        self._cmd_pub.publish(command)

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())

    def _on_estop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._estop_latched = True
        self._publish_zero()
        self._publish_emergency()
        response.success = True
        response.message = "Aegis simulation E-stop latched; /cmd_vel relay disabled"
        self.get_logger().warning(response.message)
        return response

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._estop_latched = False
        self._publish_emergency()
        response.success = True
        response.message = "Aegis simulation E-stop reset; /cmd_vel relay enabled"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Telemetry path
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _publish_health(self) -> None:
        now = time.monotonic()
        front_healthy = not is_stale(now, self._front_last_image, self._camera_stale_s)
        rear_healthy = not is_stale(now, self._rear_last_image, self._camera_stale_s)

        self._video2_health_pub.publish(Bool(data=front_healthy))
        self._webcam_health_pub.publish(Bool(data=rear_healthy))
        if front_healthy:
            primary = "video2"
        elif rear_healthy:
            primary = "webcam"
        else:
            primary = "none"
        self._primary_source_pub.publish(String(data=primary))

    def _publish_proximity(self) -> None:
        scan = self._latest_scan
        if scan is None:
            return

        distance = forward_cone_min_range(
            scan.ranges,
            float(scan.angle_min),
            float(scan.angle_increment),
            half_angle=math.radians(30.0),
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            default=float(scan.range_max),
        )
        message = Range()
        message.header = scan.header
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = math.radians(60.0)
        message.min_range = float(scan.range_min)
        message.max_range = float(scan.range_max)
        message.range = float(distance if distance is not None else scan.range_max)
        self._proximity_pub.publish(message)

    def _publish_thermal(self) -> None:
        now_s = time.monotonic()
        message = Temperature()
        message.header.stamp = self.get_clock().now().to_msg()
        message.temperature = 26.0 + 2.0 * math.sin(now_s / 20.0)
        message.variance = 0.05
        self._thermal_pub.publish(message)

    def _publish_diagnostics(self) -> None:
        now_s = time.monotonic()
        elapsed = max(0.0, now_s - self._last_diagnostics)
        self._last_diagnostics = now_s
        self._battery_percent = next_battery_percent(self._battery_percent, elapsed)

        cpu_percent = self._cpu_percent()
        memory_percent = self._memory_percent()

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            self._diagnostic_status("Battery", "aegis_sim", f"{self._battery_percent:.1f}%", "percent", f"{self._battery_percent:.1f}"),
            self._diagnostic_status("CPU", "aegis_sim", f"{cpu_percent:.1f}%", "usage", f"{cpu_percent:.1f}"),
            self._diagnostic_status("Memory", "aegis_sim", f"{memory_percent:.1f}%", "usage", f"{memory_percent:.1f}"),
        ]
        self._diagnostics_pub.publish(message)

    def _publish_emergency(self) -> None:
        self._emergency_pub.publish(Bool(data=self._estop_latched))

    @staticmethod
    def _diagnostic_status(name: str, hardware_id: str, message: str, key: str, value: str) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = name
        status.message = message
        status.hardware_id = hardware_id
        status.values = [KeyValue(key=key, value=value)]
        return status

    @staticmethod
    def _cpu_percent() -> float:
        try:
            load = os.getloadavg()[0]
        except (AttributeError, OSError):
            return 0.0
        cpus = os.cpu_count() or 1
        return max(0.0, min(100.0, 100.0 * load / cpus))

    @staticmethod
    def _memory_percent() -> float:
        try:
            values: dict[str, float] = {}
            with open("/proc/meminfo", encoding="ascii") as meminfo:
                for line in meminfo:
                    key, _, rest = line.partition(":")
                    if key in {"MemTotal", "MemAvailable"}:
                        values[key] = float(rest.split()[0])
            total = values.get("MemTotal", 0.0)
            available = values.get("MemAvailable", 0.0)
            if total > 0.0:
                return max(0.0, min(100.0, 100.0 * (total - available) / total))
        except (OSError, ValueError, IndexError):
            pass
        return 0.0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DashboardAdapter()
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
