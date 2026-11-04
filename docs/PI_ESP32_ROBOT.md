# Raspberry Pi + ESP32 robot — drive integration

This is the hardware counterpart to the Gazebo simulation: the dashboard talks to a
Raspberry Pi, the Pi talks to an ESP32 over USB serial, and the ESP32 drives four
M3508 motors over CAN.

```
Browser (dashboard, /cmd_vel + E-STOP)
        │  rosbridge ws://<pi>:9090        FastAPI :8000 (cameras, REST, config)
        ▼
Raspberry Pi: aegis_robot_bridge (ROS 2 Jazzy) ── USB serial 115200 ──▶ ESP32
                                                                         │  MCP2515 / CAN 1 Mbit
                                                                         ▼
                                                           4 × M3508 + C620 ESCs
```

The bridge implements the same contract as `aegis_dashboard_adapter` in the simulator, so
the dashboard needs no robot-specific code:

| Interface | Direction | Purpose |
| --- | --- | --- |
| `/cmd_vel` (`geometry_msgs/Twist`) | subscribe | body velocity from the dashboard (d-pad, WASD/arrow keys, Shift boost) |
| `/emergency/estop`, `/emergency/reset` (`std_srvs/Trigger`) | serve | E-STOP button and RESET button |
| `/emergency/status` (`std_msgs/Bool`) | publish | latched E-stop state |
| `/hardware/odometry` (`nav_msgs/Odometry`) | publish | dead-reckoned pose from the motor encoders |
| `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) | publish | serial link, motor RPM, commanded RPM |

`/proximity/range`, `/thermal/temperature`, `/scan`, `/camera/*` are not produced by this
bridge. `camera_bridge` publishes the camera topics; the Pi startup script sets
`RESCUE_ENABLE_CAMERA_CAPTURE=false` so the backend does not open the same devices. Lidar,
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
| `STOP` | zero RPM and keep host control for the firmware's 2 s idle window |
| `ESTOP` | latch E-stop and cut drive output |
| `RESET` | clear the latch and hold zero |
| `PING` | keep-alive, replies `OK,pong` (the bridge uses telemetry, not PING, for health) |

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

On the Pi, one command starts rosbridge, `camera_bridge`, `aegis_robot_bridge` and the
FastAPI dashboard together:

```bash
./scripts/start_pi_stack.sh
```

The script lets the drive bridge auto-detect the ESP32. Pin the device or skip a node with:

```bash
AEGIS_PORT=/dev/aegis-esp32 ./scripts/start_pi_stack.sh
SKIP_CAMERA=1 ./scripts/start_pi_stack.sh
SKIP_ROBOT=1 ./scripts/start_pi_stack.sh
```

It also sets `RESCUE_ENABLE_CAMERA_CAPTURE=false`, so `camera_bridge` is the only owner of
`/dev/video*`. The backend still serves REST, WebSockets and the frontend.

Equivalent manual terminals:

```bash
# Terminal 1: rosbridge + backend + frontend
python3 start_backend.py

# Terminal 2: camera bridge
source /opt/ros/jazzy/setup.bash
source robot_backend_ws/install/setup.bash
ros2 launch camera_bridge camera_bridge.launch.py

# Terminal 3: drive bridge
source /opt/ros/jazzy/setup.bash
source robot_backend_ws/install/setup.bash
ros2 launch aegis_robot_bridge robot.launch.py port:=/dev/aegis-esp32
```

Omit `port:=` to auto-detect. Useful overrides: `baud`, `wheel_radius`, `track_width`,
`ticks_per_wheel_rev`, `max_wheel_rpm`, `command_rate_hz`, `command_hold_s`,
`telemetry_timeout_s`.

Then open `http://<pi>:8000/` from the laptop. The launcher prints the LAN URLs; the frontend
builds `ws://<pi>:9090` from `window.location`, so no ROS cross-machine setup is needed.

### Keyboard drive map

| Key | Action |
| --- | --- |
| `↑` / `W` | forward |
| `↓` / `S` | reverse |
| `←` / `A` | turn left |
| `→` / `D` | turn right |
| `Shift` + drive key | 1.5× speed boost |
| `Esc` | software stop (publishes zero `/cmd_vel`) |
| `Q` / `E` | front-left / front-right flipper up (Ctrl = down, Shift = pair) |
| `Z` / `C` | rear-left / rear-right flipper up (Ctrl = down, Shift = pair) |
| `T` | dog stance (all flippers down); `Shift+T` returns to neutral |

The dashboard streams `/cmd_vel` at 10 Hz while a key or d-pad button is held. Speeds,
boost, stream rate and flipper step are in `frontend/ros-config.js` under `teleop`.

## 4. Safety behaviour

| Situation | Robot behaviour |
| --- | --- |
| `/cmd_vel` stops arriving (tab closed, Wi-Fi drop) for `command_hold_s` (0.35 s) | bridge sends one `STOP` and stops transmitting; wheels stop immediately |
| Dashboard idle (no key held) for about 2 s | firmware hands control back to the PS4 pad; the next dashboard key press takes over immediately |
| Bridge sees no telemetry for `telemetry_timeout_s` (1 s) | bridge sends one `STOP`, reports an ERROR diagnostic, and stops streaming |
| Serial cable unplugged / Pi reboot | firmware zeroes the wheels; after 2 s the PS4 pad can drive again |
| E-STOP pressed | firmware latches, drive output cut; `/emergency/status` = true; cleared only by RESET (dashboard RESET button) or a power cycle |
| ESP32 does not answer `ESTOP` | the bridge still latches locally, the service returns `success: false` with the reason, and the backend shows the failure |
| Node shuts down | bridge sends `STOP` once before closing the serial port |

The Pi-side bridge is the only component that sends `VEL`; nothing else should publish to
the serial port. The firmware ignores the PS4 pad while host control is active, and the
bridge intentionally stops streaming when idle so the pad fallback keeps working.

WARNING: network exposure
Rosbridge has no authentication and can publish `/cmd_vel`. Keep ports `8000` and `9090`
on a trusted LAN, or firewall them to the laptop's address; do not expose them to the
public internet.

The nav bar shows `ROBOT: ONLINE`, `ROBOT: DEGRADED`, `ROBOT: OFFLINE` or `ROBOT: E-STOP`
from the `/diagnostics` `ESP32 Link` and `Drive` entries. In simulation, where those entries
do not exist, it shows `ROBOT: N/A`.

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
| `ROBOT: OFFLINE` in the nav bar | check `ros2 topic echo /diagnostics`, the USB cable, `dialout` membership and the serial port |
| Dashboard keys do nothing but the PS4 pad drives | the bridge released host control after 2 s idle; press a key again to take over |
| Camera works in ROS but the dashboard tile is black | ensure only `camera_bridge` owns `/dev/video*` and the backend reports `local_capture_disabled` |

## Wiring summary (M3508 platform)

| Link | From | To | Notes |
| --- | --- | --- | --- |
| Motor power | 24 V battery | 4× C620 ESC → M3508 | battery GND → motor GND; fuse for worst-case stall |
| CAN H/L | MCP2515 `CANH`/`CANL` | 4× M3508 CAN pair | 120 Ω terminator at both ends of the bus |
| MCP2515 logic | ESP32 GPIO 5 (CS), 4 (INT), SPI pins (MOSI/MISO/SCK) | MCP2515 | level-shift if MCP2515 uses 5 V logic |
| Common ground | battery GND / ESP32 / CAN module | share one ground path | star ground preferred |
| ESP32 USB | ESP32 Micro-USB | Pi USB port | bridge sketch baud 115200; `/dev/aegis-esp32` after udev rule |
| Webcam | webcam USB | Pi USB port | `/dev/video*` auto-created; confirm with `v4l2-ctl --list-devices` |

Firmware defaults to match (`main.ino` in `~/Downloads/Postion_Velocity_control_robomaster_M3508_motor-main/`):
- `CAN_CS = 5`, `CAN_INT = 4`, baud `CAN_1000KBPS`, MCP2515 clock `MCP_8MHZ`
- command CAN ID `0x200`, payload order RB/LF/RF/LB (2 bytes each)
- feedback IDs: RB `0x201`, LF `0x202`, RF `0x203`, LB `0x204`
- `WHEEL_RADIUS = 0.05 m`, `TRACK_WIDTH = 0.28 m`, `TICKS_PER_MOTOR_REV = 8192`, `GEAR_RATIO = 1.0`
- `MAX_RPM = 1500`, position-mode deadband 80 ticks

WARNING: voltage levels
MCP2515 modules are commonly 5 V logic; the ESP32 GPIO are 3.3 V tolerant only. If your MCP2515 runs 5 V on `MOSI`/`MISO`/`SCK`/`INT`/`CS`, insert a level shifter on those lines or use a 3.3 V module.

WARNING: ground
ESP32 and the CAN module must share ground with the motor supply (star ground is safest). Floating ground shows up as no CAN init and missing `OK,estop`.

WARNING: current
Route the 24 V motor leads and ESC logic wires away from the CAN pair and the ESP32 USB cable — motor current induces noise in CAN and encoder feedback if they share a bundle.

For the simulator workflow see the README; for more camera detail see `docs/CONNECT_CAMERAS_AND_ROS.md`.
