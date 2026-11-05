from __future__ import annotations

import math

import pytest
from aegis_robot_bridge.protocol import (
    MAX_CURRENT_A,
    Ack,
    DriveTelemetry,
    Telemetry,
    WheelOdometry,
    format_current,
    format_estop,
    format_position,
    format_reset,
    format_velocity,
    next_current_command,
    next_drive_command,
    parse_line,
    rpm_to_mps,
    twist_to_wheel_rpm,
    yaw_to_quaternion,
)

TICKS_PER_REV = 8192.0
WHEEL_RADIUS = 0.05
TRACK_WIDTH = 0.28
METERS_PER_TICK = 2.0 * math.pi * WHEEL_RADIUS / TICKS_PER_REV


def test_format_velocity_rounds_and_clamps() -> None:
    assert format_velocity(120.4, -33.6) == "VEL,120,-34"
    assert format_velocity(99999.0, -99999.0) == "VEL,1500,-1500"
    assert format_velocity(10.0, 10.0, max_rpm=5.0) == "VEL,5,5"


def test_format_current_rounds_and_clamps() -> None:
    assert format_current(1.234, -0.5) == "CUR,1234,-500"
    assert format_current(99.0, -99.0) == "CUR,10000,-10000"
    assert format_current(2.0, -2.0, max_current_a=1.5) == "CUR,1500,-1500"
    assert MAX_CURRENT_A == 10.0


def test_format_position_rounds_and_clamps() -> None:
    assert format_position(0.05, -0.2) == "POS,50,-200"
    assert format_position(10.0, 10.0) == "POS,500,1571"
    assert format_position(-10.0, -10.0) == "POS,-500,-1571"


def test_command_lines_are_stable() -> None:
    assert format_estop() == "ESTOP"
    assert format_reset() == "RESET"
    assert parse_line("") is None
    assert parse_line("# Move cmd: dx=0.020 m") is None


def _drive(**overrides):
    values = {
        "estop_latched": False,
        "link_healthy": True,
        "command_age_s": 0.1,
        "command_hold_s": 0.35,
        "left_rpm": 100.0,
        "right_rpm": -100.0,
        "streaming": False,
    }
    values.update(overrides)
    return next_drive_command(**values)


def test_drive_command_streams_only_while_cmd_vel_is_fresh() -> None:
    line, streaming = _drive()
    assert line == "VEL,100,-100"
    assert streaming is True


def test_drive_command_sends_one_stop_when_stream_ends() -> None:
    line, streaming = _drive(command_age_s=0.5, streaming=True)
    assert line == "STOP"
    assert streaming is False

    line, streaming = _drive(command_age_s=0.5, streaming=False)
    assert line is None
    assert streaming is False


def test_drive_command_stops_for_estop_or_unhealthy_link() -> None:
    for overrides in ({"estop_latched": True}, {"link_healthy": False}):
        line, streaming = _drive(streaming=True, **overrides)
        assert line == "STOP"
        assert streaming is False

        line, streaming = _drive(streaming=False, **overrides)
        assert line is None
        assert streaming is False


def _torque(**overrides):
    values = {
        "estop_latched": False,
        "link_healthy": True,
        "command_age_s": 0.1,
        "command_hold_s": 0.35,
        "left_a": 1.5,
        "right_a": -2.0,
        "streaming": False,
    }
    values.update(overrides)
    return next_current_command(**values)


def test_torque_command_streams_only_while_cmd_current_is_fresh() -> None:
    line, streaming = _torque()
    assert line == "CUR,1500,-2000"
    assert streaming is True


def test_torque_command_sends_one_stop_when_stream_ends() -> None:
    line, streaming = _torque(command_age_s=0.5, streaming=True)
    assert line == "STOP"
    assert streaming is False

    line, streaming = _torque(command_age_s=0.5, streaming=False)
    assert line is None
    assert streaming is False


def test_parse_drive_telemetry() -> None:
    frame = parse_line("DRV,P,0,0,1200,-800,100,-60\n")
    assert frame == DriveTelemetry(
        mode="position",
        command_left_current_ma=0,
        command_right_current_ma=0,
        target_left_ticks=1200,
        target_right_ticks=-800,
        error_left_ticks=100,
        error_right_ticks=-60,
    )
    assert parse_line("DRV,T,1500,-2000,0,0,0,0\n").mode == "torque"
    assert parse_line("DRV,X,1,2,3,4,5,6\n") is None
    assert parse_line("DRV,T,1,2,3,4,5\n") is None


def test_parse_telemetry() -> None:
    frame = parse_line("TEL,12345,0,80,-80,79,-78,4096,-2048\n")
    assert frame == Telemetry(
        uptime_ms=12345,
        estop=False,
        command_left_rpm=80,
        command_right_rpm=-80,
        rpm_left=79,
        rpm_right=-78,
        ticks_left=4096,
        ticks_right=-2048,
    )


def test_parse_telemetry_rejects_truncated_frames() -> None:
    assert parse_line("TEL,1,0,0,0,0,0,0") is None
    assert parse_line("TEL,1,0,0,0,0,0,x,0") is None


def test_parse_acks() -> None:
    assert parse_line("OK,estop") == Ack(True, "estop")
    assert parse_line("ERR,unknown command") == Ack(False, "unknown command")


def test_twist_to_wheel_rpm_straight_and_turn() -> None:
    left, right = twist_to_wheel_rpm(0.5, 0.0, WHEEL_RADIUS, TRACK_WIDTH)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.5 / (2 * math.pi * WHEEL_RADIUS) * 60.0)
    assert rpm_to_mps(left, WHEEL_RADIUS) == pytest.approx(0.5)

    left, right = twist_to_wheel_rpm(0.0, 1.0, WHEEL_RADIUS, TRACK_WIDTH)
    assert left < 0.0 < right
    assert left == pytest.approx(-right)


def test_twist_to_wheel_rpm_clamps_and_validates() -> None:
    left, right = twist_to_wheel_rpm(50.0, 0.0, WHEEL_RADIUS, TRACK_WIDTH)
    assert (left, right) == (1500.0, 1500.0)

    with pytest.raises(ValueError):
        twist_to_wheel_rpm(0.0, 0.0, 0.0, TRACK_WIDTH)
    with pytest.raises(ValueError):
        twist_to_wheel_rpm(0.0, 0.0, WHEEL_RADIUS, -1.0)


def test_odometry_straight_line() -> None:
    odom = WheelOdometry(WHEEL_RADIUS, TRACK_WIDTH, TICKS_PER_REV)
    odom.update(0, 0)
    ticks = int(round(1.0 / METERS_PER_TICK))

    pose = odom.update(ticks, ticks)

    assert pose.x == pytest.approx(1.0, rel=1e-3)
    assert pose.y == pytest.approx(0.0, abs=1e-9)
    assert pose.yaw == pytest.approx(0.0, abs=1e-9)


def test_odometry_in_place_rotation() -> None:
    odom = WheelOdometry(WHEEL_RADIUS, TRACK_WIDTH, TICKS_PER_REV)
    odom.update(0, 0)
    arc = 0.2 * TRACK_WIDTH / 2.0
    ticks = int(round(arc / METERS_PER_TICK))

    pose = odom.update(-ticks, ticks)

    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.yaw == pytest.approx(0.2, rel=5e-2)


def test_odometry_rebaselines_on_encoder_jump() -> None:
    odom = WheelOdometry(WHEEL_RADIUS, TRACK_WIDTH, TICKS_PER_REV)
    odom.update(0, 0)
    ticks = int(round(1.0 / METERS_PER_TICK))
    odom.update(ticks, ticks)
    before = odom.pose.x

    pose = odom.update(ticks + 10 * int(TICKS_PER_REV), ticks + 10 * int(TICKS_PER_REV))

    assert pose.x == pytest.approx(before)
    assert pose.y == pytest.approx(0.0, abs=1e-9)


def test_yaw_to_quaternion() -> None:
    assert yaw_to_quaternion(0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    x, y, z, w = yaw_to_quaternion(math.pi / 2.0)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))
