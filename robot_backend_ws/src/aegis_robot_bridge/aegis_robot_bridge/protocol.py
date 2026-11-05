"""Wire protocol and drivetrain math for the ESP32 motor bridge.

The ESP32 (``firmware/esp32_motor_bridge``) speaks newline-delimited ASCII over
USB serial at 115200 baud.

Host -> ESP32::

    VEL,<left_rpm>,<right_rpm>   wheel RPM, positive = forward (unacknowledged)
    CUR,<left_mA>,<right_mA>     direct current, clamped to +/-10000 mA
    POS,<dx_mm>,<dtheta_mrad>    relative position move (first POS holds)
    STOP                         zero drive output, return host to velocity
    ESTOP                        latch emergency stop, cut drive output
    RESET                        clear the latch and hold zero
    PING                         keep-alive (stream while in position mode)

ESP32 -> Host::

    TEL,<ms>,<estop>,<cmd_l>,<cmd_r>,<rpm_l>,<rpm_r>,<ticks_l>,<ticks_r>
    DRV,<T|P>,<cmd_l>,<cmd_r>,<tgt_l>,<tgt_r>,<err_l>,<err_r>
    OK,<message>
    ERR,<message>

``ticks_*`` are accumulated encoder ticks per *side*, already sign-normalised so
positive means forward; the firmware averages the two motors on each side.
Any other line the firmware prints (human-readable logs) is ignored by the
parser, so existing ``Serial.println`` output stays valid.

This module is deliberately free of ROS, pyserial, and hardware imports so the
codec and kinematics can be unit tested without a robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

BAUD_RATE = 115200
MAX_WHEEL_RPM = 1500.0
MAX_CURRENT_A = 10.0
MAX_POSITION_DX_M = 0.5
MAX_POSITION_DTHETA_RAD = 1.571


@dataclass(frozen=True)
class Telemetry:
    """One ``TEL`` frame from the ESP32."""

    uptime_ms: int
    estop: bool
    command_left_rpm: int
    command_right_rpm: int
    rpm_left: int
    rpm_right: int
    ticks_left: int
    ticks_right: int


@dataclass(frozen=True)
class DriveTelemetry:
    """One ``DRV`` frame from a host CURRENT/POSITION session."""

    mode: str
    command_left_current_ma: int
    command_right_current_ma: int
    target_left_ticks: int
    target_right_ticks: int
    error_left_ticks: int
    error_right_ticks: int


@dataclass(frozen=True)
class Ack:
    """``OK``/``ERR`` reply to an acknowledged command."""

    ok: bool
    message: str


def clamp_rpm(value: float, max_rpm: float = MAX_WHEEL_RPM) -> float:
    limit = abs(float(max_rpm))
    return max(-limit, min(limit, float(value)))


def format_velocity(left_rpm: float, right_rpm: float, max_rpm: float = MAX_WHEEL_RPM) -> str:
    left = int(round(clamp_rpm(left_rpm, max_rpm)))
    right = int(round(clamp_rpm(right_rpm, max_rpm)))
    return f"VEL,{left},{right}"


def clamp_current(value: float, max_current_a: float = MAX_CURRENT_A) -> float:
    limit = abs(float(max_current_a))
    return max(-limit, min(limit, float(value)))


def format_current(left_a: float, right_a: float, max_current_a: float = MAX_CURRENT_A) -> str:
    limit_ma = int(round(abs(float(max_current_a)) * 1000.0))
    left = int(round(clamp_current(left_a, max_current_a) * 1000.0))
    right = int(round(clamp_current(right_a, max_current_a) * 1000.0))
    left = max(-limit_ma, min(limit_ma, left))
    right = max(-limit_ma, min(limit_ma, right))
    return f"CUR,{left},{right}"


def format_position(dx_m: float, dtheta_rad: float) -> str:
    dx = max(-MAX_POSITION_DX_M, min(MAX_POSITION_DX_M, float(dx_m)))
    dtheta = max(-MAX_POSITION_DTHETA_RAD, min(MAX_POSITION_DTHETA_RAD, float(dtheta_rad)))
    return f"POS,{int(round(dx * 1000.0))},{int(round(dtheta * 1000.0))}"


def format_stop() -> str:
    return "STOP"


def format_estop() -> str:
    return "ESTOP"


def format_reset() -> str:
    return "RESET"


def format_ping() -> str:
    return "PING"


def next_drive_command(
    *,
    estop_latched: bool,
    link_healthy: bool,
    command_age_s: float,
    command_hold_s: float,
    left_rpm: float,
    right_rpm: float,
    max_rpm: float = MAX_WHEEL_RPM,
    streaming: bool,
) -> tuple[str | None, bool]:
    """Return ``(serial_line, streaming)`` for one command tick.

    The ESP32 treats every host line as a sign of life: it ignores the PS4 pad
    while host control is active and only hands control back after 2 s without
    a command. Streaming ``VEL,0,0`` while the dashboard is idle would keep the
    pad locked out forever, so only stream while a fresh ``/cmd_vel`` exists and
    send a single ``STOP`` when that stream ends.
    """
    active = link_healthy and not estop_latched and command_age_s <= command_hold_s
    if active:
        return format_velocity(left_rpm, right_rpm, max_rpm), True
    if streaming:
        return format_stop(), False
    return None, False


def next_current_command(
    *,
    estop_latched: bool,
    link_healthy: bool,
    command_age_s: float,
    command_hold_s: float,
    left_a: float,
    right_a: float,
    max_current_a: float = MAX_CURRENT_A,
    streaming: bool,
) -> tuple[str | None, bool]:
    """Return ``(serial_line, streaming)`` for one torque command tick."""
    active = link_healthy and not estop_latched and command_age_s <= command_hold_s
    if active:
        return format_current(left_a, right_a, max_current_a), True
    if streaming:
        return format_stop(), False
    return None, False


def parse_line(line: str) -> Telemetry | DriveTelemetry | Ack | None:
    """Parse one serial line. Returns ``None`` for logs, blank lines, or garbage."""
    text = line.strip()
    if not text:
        return None
    if text.startswith("TEL,"):
        return _parse_telemetry(text)
    if text.startswith("DRV,"):
        return _parse_drive_telemetry(text)
    if text.startswith("OK,"):
        return Ack(True, text[3:].strip())
    if text.startswith("ERR,"):
        return Ack(False, text[4:].strip())
    return None


def _parse_telemetry(text: str) -> Telemetry | None:
    fields = text.split(",")
    if len(fields) != 9:
        return None
    try:
        uptime_ms = int(fields[1])
        estop = int(fields[2]) != 0
        command_left_rpm = int(fields[3])
        command_right_rpm = int(fields[4])
        rpm_left = int(fields[5])
        rpm_right = int(fields[6])
        ticks_left = int(fields[7])
        ticks_right = int(fields[8])
    except (TypeError, ValueError):
        return None
    return Telemetry(
        uptime_ms=uptime_ms,
        estop=estop,
        command_left_rpm=command_left_rpm,
        command_right_rpm=command_right_rpm,
        rpm_left=rpm_left,
        rpm_right=rpm_right,
        ticks_left=ticks_left,
        ticks_right=ticks_right,
    )


def _parse_drive_telemetry(text: str) -> DriveTelemetry | None:
    fields = text.split(",")
    if len(fields) != 8:
        return None
    mode = {"T": "torque", "P": "position"}.get(fields[1])
    if mode is None:
        return None
    try:
        command_left_current_ma = int(fields[2])
        command_right_current_ma = int(fields[3])
        target_left_ticks = int(fields[4])
        target_right_ticks = int(fields[5])
        error_left_ticks = int(fields[6])
        error_right_ticks = int(fields[7])
    except (TypeError, ValueError):
        return None
    return DriveTelemetry(
        mode=mode,
        command_left_current_ma=command_left_current_ma,
        command_right_current_ma=command_right_current_ma,
        target_left_ticks=target_left_ticks,
        target_right_ticks=target_right_ticks,
        error_left_ticks=error_left_ticks,
        error_right_ticks=error_right_ticks,
    )


def twist_to_wheel_rpm(
    linear_x: float,
    angular_z: float,
    wheel_radius: float,
    track_width: float,
    max_rpm: float = MAX_WHEEL_RPM,
) -> tuple[float, float]:
    """Convert ``geometry_msgs/Twist`` body velocities (m/s, rad/s) to wheel RPM."""
    if wheel_radius <= 0.0:
        raise ValueError("wheel_radius must be positive")
    if track_width < 0.0:
        raise ValueError("track_width must not be negative")

    half_track = track_width / 2.0
    left_mps = float(linear_x) - float(angular_z) * half_track
    right_mps = float(linear_x) + float(angular_z) * half_track
    meters_per_rev = 2.0 * math.pi * wheel_radius
    left_rpm = left_mps / meters_per_rev * 60.0
    right_rpm = right_mps / meters_per_rev * 60.0
    return clamp_rpm(left_rpm, max_rpm), clamp_rpm(right_rpm, max_rpm)


def rpm_to_mps(rpm: float, wheel_radius: float) -> float:
    """Wheel surface speed in m/s for a wheel RPM."""
    return float(rpm) / 60.0 * 2.0 * math.pi * wheel_radius


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class WheelOdometry:
    """Dead-reckons a planar pose from side-normalised encoder ticks."""

    def __init__(
        self,
        wheel_radius: float,
        track_width: float,
        ticks_per_wheel_rev: float,
        max_delta_ticks: float | None = None,
    ) -> None:
        if wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")
        if track_width <= 0.0:
            raise ValueError("track_width must be positive")
        if ticks_per_wheel_rev <= 0.0:
            raise ValueError("ticks_per_wheel_rev must be positive")

        self.wheel_radius = float(wheel_radius)
        self.track_width = float(track_width)
        self.ticks_per_wheel_rev = float(ticks_per_wheel_rev)
        # A jump this large can only be a firmware reboot or a slipped encoder,
        # so re-baseline instead of teleporting the pose.
        self.max_delta_ticks = float(max_delta_ticks) if max_delta_ticks is not None else 4.0 * self.ticks_per_wheel_rev
        self._last_left: int | None = None
        self._last_right: int | None = None
        self.pose = Pose2D()

    def reset(self) -> None:
        self._last_left = None
        self._last_right = None
        self.pose = Pose2D()

    def update(self, ticks_left: int, ticks_right: int) -> Pose2D:
        left = int(ticks_left)
        right = int(ticks_right)
        if self._last_left is None or self._last_right is None:
            self._last_left, self._last_right = left, right
            return Pose2D(self.pose.x, self.pose.y, self.pose.yaw)

        delta_left = left - self._last_left
        delta_right = right - self._last_right
        self._last_left, self._last_right = left, right
        if abs(delta_left) > self.max_delta_ticks or abs(delta_right) > self.max_delta_ticks:
            return Pose2D(self.pose.x, self.pose.y, self.pose.yaw)

        meters_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_wheel_rev
        left_m = delta_left * meters_per_tick
        right_m = delta_right * meters_per_tick
        distance = (left_m + right_m) / 2.0
        heading = (right_m - left_m) / self.track_width

        # Midpoint integration keeps straight lines straight and turns centred.
        mid_heading = self.pose.yaw + heading / 2.0
        self.pose.x += distance * math.cos(mid_heading)
        self.pose.y += distance * math.sin(mid_heading)
        self.pose.yaw = _wrap_angle(self.pose.yaw + heading)
        return Pose2D(self.pose.x, self.pose.y, self.pose.yaw)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return ``(x, y, z, w)`` for a planar yaw rotation."""
    half = _wrap_angle(yaw) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)
