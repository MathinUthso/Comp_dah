from __future__ import annotations

import math

from aegis_sim.math_helpers import (
    forward_cone_min_range,
    is_stale,
    next_battery_percent,
    normalize_angle,
    valid_range,
)


def test_valid_range_rejects_invalid_samples() -> None:
    assert valid_range(1.25, 0.1, 12.0) == 1.25
    assert valid_range(float("inf"), 0.1, 12.0) is None
    assert valid_range(float("nan"), 0.1, 12.0) is None
    assert valid_range(0.05, 0.1, 12.0) is None
    assert valid_range("bad", 0.1, 12.0) is None


def test_forward_cone_only_uses_front_samples() -> None:
    ranges = [5.0, 0.4, 0.1, 5.0, 5.0]
    angle_min = -0.2
    angle_increment = 0.2

    assert forward_cone_min_range(ranges, angle_min, angle_increment, half_angle=0.1) == 0.4


def test_forward_cone_skips_invalid_and_returns_default() -> None:
    ranges = [float("nan"), float("inf"), 20.0]
    assert forward_cone_min_range(ranges, -0.1, 0.1, range_min=0.1, range_max=12.0) is None
    assert forward_cone_min_range(ranges, -0.1, 0.1, range_min=0.1, range_max=12.0, default=12.0) == 12.0


def test_normalize_angle_wraps() -> None:
    assert normalize_angle(0.0) == 0.0
    assert abs(normalize_angle(2 * math.pi) - 0.0) < 1e-9
    assert abs(normalize_angle(-2 * math.pi) - 0.0) < 1e-9


def test_battery_drain_is_bounded() -> None:
    assert next_battery_percent(50.0, 10.0, linear_x=1.0) < 50.0
    assert next_battery_percent(0.5, 1000.0) == 0.0
    assert next_battery_percent(99.9, 0.0) == 99.9


def test_stale_timestamp() -> None:
    assert is_stale(10.0, 0.0, 1.0)
    assert is_stale(10.0, 8.0, 1.0)
    assert not is_stale(10.0, 9.5, 1.0)
