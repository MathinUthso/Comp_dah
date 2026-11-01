"""Pure math helpers shared by the Aegis simulation adapter and tests."""

from __future__ import annotations

import math
from collections.abc import Iterable

TWO_PI = 2.0 * math.pi


def valid_range(value: object, range_min: float = 0.0, range_max: float = 12.0) -> float | None:
    """Return a finite in-range lidar distance, or ``None`` for an invalid sample."""
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(distance):
        return None
    if distance < range_min or distance > range_max:
        return None
    return distance


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % TWO_PI - math.pi


def forward_cone_min_range(
    ranges: Iterable[object],
    angle_min: float,
    angle_increment: float,
    half_angle: float = math.radians(30.0),
    range_min: float = 0.0,
    range_max: float = 12.0,
    default: float | None = None,
) -> float | None:
    """Smallest valid lidar range inside a forward-facing cone."""
    if angle_increment == 0:
        return default

    minimum: float | None = None
    for index, value in enumerate(ranges):
        angle = normalize_angle(angle_min + index * angle_increment)
        if abs(angle) > half_angle:
            continue
        distance = valid_range(value, range_min, range_max)
        if distance is None:
            continue
        minimum = distance if minimum is None else min(minimum, distance)
    return minimum if minimum is not None else default


def next_battery_percent(
    current: float,
    elapsed_s: float,
    linear_x: float = 0.0,
    angular_z: float = 0.0,
) -> float:
    """Drain the synthetic battery based on commanded motion and idle time."""
    elapsed = max(0.0, float(elapsed_s))
    drain = elapsed * (0.002 + 0.02 * abs(float(linear_x)) + 0.01 * abs(float(angular_z)))
    return max(0.0, min(100.0, float(current) - drain))


def is_stale(now_s: float, last_s: float, timeout_s: float) -> bool:
    """True when a monotonic timestamp is older than the timeout."""
    return last_s <= 0.0 or (now_s - last_s) > max(0.0, float(timeout_s))
