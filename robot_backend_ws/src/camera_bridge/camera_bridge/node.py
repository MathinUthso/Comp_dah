from __future__ import annotations

import glob
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import cv2
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
except Exception:  # pragma: no cover - ROS2 runtime only
    cv2 = None
    rclpy = None
    Node = object  # type: ignore
    CompressedImage = Bool = String = object  # type: ignore


@dataclass
class SourceRuntime:
    name: str
    topic_prefix: str
    device_param: str
    device: str | None = None
    capture: Any | None = None
    healthy: bool = False
    last_error: str = ""


class CameraBridgeNode(Node):
    SOURCE_HINTS = {
        "video2": ("hdmi", "capture", "cam link", "uvc", "video capture"),
        "webcam": ("webcam", "camera", "usb video"),
        "thermal": ("thermal", "ir", "flir", "seek", "infrared"),
    }

    def __init__(self) -> None:
        super().__init__("camera_bridge")
        self.declare_parameter("scan_glob", "/dev/video*")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("video2_device", "")
        self.declare_parameter("webcam_device", "")
        self.declare_parameter("thermal_device", "")

        self._source_order = ("video2", "webcam", "thermal")
        self._sources = {
            "video2": SourceRuntime("video2", "/camera/video2", "video2_device"),
            "webcam": SourceRuntime("webcam", "/camera/webcam", "webcam_device"),
            "thermal": SourceRuntime("thermal", "/camera/thermal", "thermal_device"),
        }
        self._image_publishers = {
            name: self.create_publisher(CompressedImage, f"{runtime.topic_prefix}/image_compressed", 10)
            for name, runtime in self._sources.items()
        }
        self._health_publishers = {
            name: self.create_publisher(Bool, f"{runtime.topic_prefix}/healthy", 10)
            for name, runtime in self._sources.items()
        }
        self._primary_publisher = self.create_publisher(String, "/camera/primary_source", 10)
        self._generic_publisher = self.create_publisher(CompressedImage, "/camera/image_compressed", 10)
        self._current_primary = "none"
        self._timer = self.create_timer(1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0), self._tick)
        self._auto_discovered: dict[str, str | None] = self._discover_devices()
        self.get_logger().info(f"camera bridge discovered devices: {self._auto_discovered}")

    def destroy_node(self):  # type: ignore[override]
        for runtime in self._sources.values():
            self._release_capture(runtime)
        return super().destroy_node()

    def _discover_devices(self) -> dict[str, str | None]:
        scan_glob = str(self.get_parameter("scan_glob").value)
        candidates: list[tuple[str, str]] = []
        for path in sorted(glob.glob(scan_glob)):
            name_file = Path("/sys/class/video4linux") / Path(path).name / "name"
            try:
                label = name_file.read_text().strip()
            except Exception:
                label = "unknown"
            candidates.append((path, label))

        used: set[str] = set()
        discovered: dict[str, str | None] = {}
        for source_name in self._source_order:
            explicit = str(self.get_parameter(self._sources[source_name].device_param).value).strip()
            if explicit:
                discovered[source_name] = explicit
                used.add(explicit)
                continue

            match = self._match_candidate(source_name, candidates, used)
            discovered[source_name] = match
            if match:
                used.add(match)

        return discovered

    def _match_candidate(self, source_name: str, candidates: list[tuple[str, str]], used: set[str]) -> str | None:
        hints = self.SOURCE_HINTS[source_name]
        for device, label in candidates:
            if device in used:
                continue
            lowered = label.lower()
            if any(hint in lowered for hint in hints):
                return device
        for device, _label in candidates:
            if device not in used:
                return device
        return None

    def _open_capture(self, device: str):
        if cv2 is None:
            raise RuntimeError("cv2 is unavailable in this environment")
        capture = cv2.VideoCapture(device)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("frame_width").value))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("frame_height").value))
        return capture

    def _release_capture(self, runtime: SourceRuntime) -> None:
        if runtime.capture is not None:
            try:
                runtime.capture.release()
            except Exception:
                pass
        runtime.capture = None

    def _ensure_capture(self, runtime: SourceRuntime) -> bool:
        if runtime.device is None:
            return False
        if runtime.capture is not None:
            return True
        try:
            runtime.capture = self._open_capture(runtime.device)
            return bool(runtime.capture and runtime.capture.isOpened())
        except Exception as exc:
            runtime.last_error = str(exc)
            self.get_logger().warn(f"{runtime.name} open failed: {exc}")
            self._release_capture(runtime)
            return False

    def _publish_health(self, runtime: SourceRuntime, healthy: bool) -> None:
        msg = Bool()
        msg.data = bool(healthy)
        self._health_publishers[runtime.name].publish(msg)

    def _publish_primary(self, source_name: str) -> None:
        msg = String()
        msg.data = source_name
        self._primary_publisher.publish(msg)

    def _encode_frame(self, frame):
        if cv2 is None:
            return None
        quality = int(self.get_parameter("jpeg_quality").value)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, max(1, min(95, quality))])
        if not ok:
            return None
        return encoded.tobytes()

    def _publish_frame(self, runtime: SourceRuntime, frame) -> None:
        encoded = self._encode_frame(frame)
        if encoded is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = runtime.name
        msg.format = "jpeg"
        msg.data = encoded
        self._image_publishers[runtime.name].publish(msg)
        if runtime.name == self._current_primary:
            self._generic_publisher.publish(msg)

    def _tick(self) -> None:
        primary = "none"
        for source_name in self._source_order:
            runtime = self._sources[source_name]
            if runtime.device is None:
                runtime.device = self._auto_discovered.get(source_name)

            if not runtime.device or not self._ensure_capture(runtime):
                runtime.healthy = False
                self._publish_health(runtime, False)
                continue

            ok, frame = runtime.capture.read() if runtime.capture is not None else (False, None)
            if not ok or frame is None:
                runtime.healthy = False
                runtime.last_error = f"read failed on {runtime.device}"
                self._release_capture(runtime)
                self._publish_health(runtime, False)
                continue

            runtime.healthy = True
            self._publish_health(runtime, True)
            if primary == "none":
                primary = source_name
            self._current_primary = primary
            self._publish_frame(runtime, frame)

        self._current_primary = primary
        self._publish_primary(primary)


def main() -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is unavailable in this environment")
    rclpy.init(args=None)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
