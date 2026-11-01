# Raspberry Pi + ESP32 robot — drive integration

This is the hardware counterpart to the Gazebo simulation: the dashboard talks to a
Raspberry Pi, the Pi talks to an ESP32 over USB serial, and the ESP32 drives four
M3508 motors over CAN.

```
Browser (dashboard, /cmd_vel + E-STOP)
        │  rosbridge ws://<pi>:9090        FastAPI :8000 (cameras, REST, config)
        ▼
Raspberry Pi: aegis_robot_bridge (ROS 2 Humble) ── USB serial 115200 ──▶ ESP32
                                                                         │  MCP2515 / CAN 1 Mbit
                                                                         ▼
                                                           4 × M3508 + C620 ESCs
```

The bridge implements the same contract as `aegis_dashboard_adapter` in the simulator, so
the dashboard needs no robot-specific code:

| Interface | Direction | Purpose |
| --- | --- | --- |
| `/cmd_vel` (`geometry_msgs/Twist`) | subscribe | body velocity from the dashboard (d-pad, arrow keys) |
| `/emergency/estop`, `/emergency/reset` (`std_srvs/Trigger`) | serve | E-STOP button and RESET button |
| `/emergency/status` (`std_msgs/Bool`) | publish | latched E-stop state |
| `/hardware/odometry` (`nav_msgs/Odometry`) | publish | dead-reckoned pose from the motor encoders |
| `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) | publish | serial link, motor RPM, commanded RPM |

`/proximity/range`, `/thermal/temperature`, `/scan`, `/camera/*` are not produced by this
bridge. Keep using `camera_bridge` (or the backend's `/dev/video*` path) for video; lidar,
range and thermal come from whatever sensors you add.

## 1. Firmware

The sketch in `firmware/esp32_motor_bridge/esp32_motor_bridge.ino` is the original
`main.ino` (position/velocity control, PS4 pad) plus a host protocol and telemetry.

Arduino libraries: `mcp_can` and `PS4Controller` (same as before). Inputs stay on the same
pins (`CAN_CS = 5`, `CAN_INT = 4`).

Constants you may need to match to your chassis (top of the sketch, mirrored by the launch
arguments):

| Constant | Default | Launch argument |
| --- | --- | --- |
| `WHEEL_RADIUS` | `0.05` m | `wheel_radius` |
| `TRACK_WIDTH` | `0.28` m | `track_width` |
| `TICKS_PER_MOTOR_REV * GEAR_RATIO` | `8192 * 1.0` | `ticks_per_wheel_rev` |
| `MAX_RPM` | `1500` | `max_wheel_rpm` |

### Serial protocol

Newline-delimited ASCII at 115200 baud. The Pi side lives in
`robot_backend_ws/src/aegis_robot_bridge/aegis_robot_bridge/protocol.py`.

| Pi → ESP32 | Meaning |
| --- | --- |
| `VEL,<left_rpm>,<right_rpm>` | side wheel RPM, positive = forward; implies velocity mode; unacknowledged (20 Hz) |
| `STOP` | zero RPM, keep host control |
| `ESTOP` | latch E-stop and cut drive output |
| `RESET` | clear the latch and hold zero |
| `PING` | keep-alive, replies `OK,pong` |

| ESP32 → Pi | Meaning |
| --- | --- |
| `TEL,<ms>,<estop>,<cmd_l>,<cmd_r>,<rpm_l>,<rpm_r>,<ticks_l>,<ticks_r>` | 20 Hz telemetry; RPM/ticks are side-normalised (positive = forward) |
| `OK,<message>` / `ERR,<message>` | reply to `STOP`/`ESTOP`/`RESET`/`PING` |

Lines the firmware prints for humans (`# ...`) are ignored by the Pi, so debug prints are safe.

## 2. Raspberry Pi setup (Ubuntu 24.04 / ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-ros-base python3-serial python3-venv
sudo usermod -aG dialout "$USER"          # log out / back in afterwards
```

On Ubuntu 22.04 use `ros-humble-ros-base` and the `main` branch instead; the commands below are
the same apart from the distro name.

Build the workspace (the Pi needs no other new dependency):

```bash
source /opt/ros/jazzy/setup.bash
cd robot_backend_ws
colcon build --packages-select dashboard_bridge camera_bridge aegis_description aegis_robot_bridge
source install/setup.bash
```

`aegis_sim`/`aegis_gazebo_plugins` are skipped on 24.04 (Gazebo Classic 11 is 22.04-only, and both
directories carry a `COLCON_IGNORE` on this branch).

### Stable serial device name

```bash
sudo cp robot_backend_ws/src/aegis_robot_bridge/deploy/99-aegis-esp32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/aegis-esp32
```

Without the rule the bridge auto-detects: it prefers `/dev/serial/by-id/*` names for common
ESP32 adapters (CP210x, CH340/CH910, native USB-CDC) and falls back to `/dev/ttyUSB*` /
`/dev/ttyACM*`.

## 3. Run

Terminal 1 — dashboard backend, rosbridge and frontend (unchanged):

```bash
python3 start_backend.py
```

Terminal 2 — drive bridge:

```bash
source /opt/ros/jazzy/setup.bash
source robot_backend_ws/install/setup.bash
ros2 launch aegis_robot_bridge robot.launch.py port:=/dev/aegis-esp32
```

Omit `port:=` to auto-detect. Useful overrides: `baud`, `wheel_radius`, `track_width`,
`ticks_per_wheel_rev`, `max_wheel_rpm`, `command_rate_hz`, `command_hold_s`,
`telemetry_timeout_s`.

Then open `http://<pi>:8000/`: the d-pad, arrow keys and the E-STOP button drive the robot.

## 4. Safety behaviour

| Situation | Robot behaviour |
| --- | --- |
| `/cmd_vel` stops arriving (tab closed, Wi-Fi drop) for `command_hold_s` (0.35 s) | bridge streams `VEL,0,0`; wheels stop |
| Bridge sees no telemetry for `telemetry_timeout_s` (1 s) | bridge streams zero and reports an ERROR diagnostic; firmware also zeroes after 400 ms of silence |
| Serial cable unplugged / Pi reboot | firmware zeroes the wheels; after 2 s the PS4 pad can drive again |
| E-STOP pressed | firmware latches, drive output cut; `/emergency/status` = true; cleared only by RESET (dashboard RESET button) or a power cycle |
| ESP32 does not answer `ESTOP` | the bridge still latches locally, the service returns `success: false` with the reason, and the backend shows the failure |

The Pi-side bridge is the only component that sends `VEL`; nothing else should publish to
the serial port.

## 5. Position mode

The firmware keeps its incremental position mode (`L1` on the PS4 pad, `Circle` to stop,
`Triangle` to print motor status). While the dashboard is driving, the firmware forces
velocity mode; position mode is currently local to the pad and not exposed through ROS.

## 6. Troubleshooting

| Symptom | Check |
| --- | --- |
| `serial link down (Permission denied)` | user in `dialout`, then re-login; or use the udev rule with `MODE="0666"` |
| `no serial device found` | `ls /dev/serial/by-id/`, `dmesg | tail` after plugging the ESP32; pass `port:=` explicitly |
| Link up, `no motor telemetry yet` | wrong port (another tty), wrong baud, or firmware not flashed with the host protocol |
| E-STOP returns failure | ESP32 not answering `ESTOP`; check the serial monitor for `OK,estop` / `ERR,unknown` |
| Robot drives the wrong way | motor CAN IDs / mounting; flip that side in `applySideVelocity()` in the sketch |
| Odometry drifts or spins | set `wheel_radius`, `track_width`, `ticks_per_wheel_rev` to the real values; wheel slip is not observable |
| Robot stops after a second | `/cmd_vel` is not being streamed — keep the d-pad/key held, or check `command_hold_s` |

For the simulator workflow see the README; for camera wiring see
`docs/CONNECT_CAMERAS_AND_ROS.md`.
