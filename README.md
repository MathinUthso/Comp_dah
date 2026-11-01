# Rescue robot dashboard backend

> **Branch `ubuntu-24-jazzy`:** targets Ubuntu 24.04, ROS 2 Jazzy and Python 3.12. The Gazebo
> Classic simulation is Humble/22.04-only and lives on `main`; `aegis_sim` and
> `aegis_gazebo_plugins` carry a `COLCON_IGNORE` here so the rest of the workspace builds.

## Architecture

### 1. FastAPI
FastAPI handles operator-facing REST APIs, WebSocket fan-out, camera routing, config management, Lab_View engineering tools, diagnostics aggregation, and hardware-control orchestration.

### 2. rosbridge
rosbridge provides dynamic ROS graph inspection, topic/service access, and non-critical pub/sub integration for telemetry, FlexBE introspection, and engineering tools.

### 3. Direct ROS2 node communication
Direct `rclpy` calls are reserved for critical paths: E-STOP and emergency reset (`std_srvs/Trigger` services) and motor halt (zero `Twist` on `/cmd_vel`).

## Folder structure

frontend/               # static dashboard (HTML/CSS/JS, served by FastAPI)

backend/
├── api/
├── websocket/
├── ros_bridge/
├── flexbe_manager/
├── camera_manager/
├── thermal_manager/
├── logs/
├── diagnostics/
├── config/
├── mission_control/
├── hardware_interface/
├── state_sync/
├── debug/
├── labview/
└── utils/

robot_backend_ws/       # canonical ROS 2 workspace (build here)
└── src/
    ├── dashboard_bridge/  # all ROS node entrypoints + launch file
    ├── camera_bridge/     # camera capture/publication
    ├── aegis_description/ # Aegis URDF/xacro (RViz + Gazebo Classic)
    ├── aegis_gazebo_plugins/ # C++ flipper-controller Gazebo plugin
    ├── aegis_robot_bridge/   # Raspberry Pi <-> ESP32 drive bridge
    └── aegis_sim/         # Gazebo world, spawner, dashboard adapter

firmware/               # ESP32 Arduino sketch for the real M3508 robot
docs/                   # integration manuals

## Deployment steps

1. Install Python dependencies from requirements.txt (the repo has a `.venv`).
2. Build the ROS 2 workspace:
   `cd robot_backend_ws && colcon build --packages-select dashboard_bridge camera_bridge aegis_description aegis_robot_bridge`.
3. Source ROS 2 Jazzy and the workspace overlay: `source /opt/ros/jazzy/setup.bash && source robot_backend_ws/install/setup.bash`.
4. Start the dashboard launcher script or start rosbridge plus uvicorn manually.
5. Open the dashboard at http://127.0.0.1:8000/.

### Recommended startup

- `python3 start_backend.py`

This launches rosbridge, the backend API, and serves the frontend from the same origin.

## Runtime notes

- Camera assignment is dynamic and device-driven.
- Thermal rotation and flip correction are live-configurable from Lab_View.
- WebSocket streams are persistent and push-based.
- All critical emergency actions bypass rosbridge and use direct `rclpy` publishers.
- The frontend now connects directly to backend REST and WebSocket endpoints.
- Lab_View opens the engineering panel for camera routing, ROS graph tools, and diagnostics.

## Gazebo Classic simulation

**Humble branch only.** Gazebo Classic 11 has no Ubuntu 24.04 packages, so the simulator is not
available on `ubuntu-24-jazzy`; check out `main` on an Ubuntu 22.04 machine for this section.

The Aegis robot can run from the same URDF in Gazebo Classic 11 and feed the dashboard without
changing the hardware camera path.

```bash
# Terminal 1: rosbridge + backend + frontend
python3 start_backend.py

# Terminal 2: Gazebo + robot + dashboard adapter
source /opt/ros/jazzy/setup.bash
source robot_backend_ws/install/setup.bash
ros2 launch aegis_sim sim.launch.py
```

The launch file starts Gazebo, `robot_state_publisher`, the model spawner, and
`aegis_dashboard_adapter`. It does **not** start a second rosbridge. Use `gui:=false` for a
headless run and `rviz:=true` to inspect the model and lidar in RViz.

Simulation topic contract:

- `/cmd_vel` is relayed by the adapter to `/cmd_vel_sim`; the Gazebo diff-drive plugin drives only
  the front wheel pair.
- `/hardware/odometry`, `/hardware/imu`, `/scan`, `/camera/front/image_raw`, and
  `/camera/rear/image_raw` come from Gazebo.
- `/camera/image_compressed`, `/camera/video2/image_compressed`, `/camera/webcam/image_compressed`,
  camera health, `/proximity/range`, `/thermal/temperature`, `/diagnostics`, and emergency status
  are produced by the adapter.
- `/emergency/estop` and `/emergency/reset` are implemented as `std_srvs/Trigger` services by the
  adapter. E-stop latches the relay and publishes zero velocity to `/cmd_vel_sim`.

The dashboard's **Lidar** tab renders `/scan` as a live polar plot. Front/rear camera tiles prefer
ROS compressed images when available and fall back to the existing MJPEG endpoints. CAM_03/CAM_04
show `NO SIGNAL` in simulation because only two simulated cameras are provided.

## Raspberry Pi + ESP32 hardware robot

The dashboard can drive the real robot: an ESP32 runs the M3508 motor control firmware
(`firmware/esp32_motor_bridge`) and the Pi runs `aegis_robot_bridge`, which streams
`/cmd_vel` over USB serial and publishes wheel odometry, E-stop status and diagnostics.
The dashboard keeps using the same topics and the same E-STOP/RESET buttons.

```bash
# Terminal 1: rosbridge + backend + frontend
python3 start_backend.py

# Terminal 2: Pi drive bridge (firmware flashed, ESP32 plugged in)
source /opt/ros/jazzy/setup.bash
source robot_backend_ws/install/setup.bash
ros2 launch aegis_robot_bridge robot.launch.py port:=/dev/aegis-esp32
```

See docs/PI_ESP32_ROBOT.md for wiring, firmware, serial protocol, safety behaviour, and
troubleshooting.

## Integration manual

See docs/CONNECT_CAMERAS_AND_ROS.md for full camera wiring, ROS workspace connection, startup, Lab_View operations, and troubleshooting steps.
