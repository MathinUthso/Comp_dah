# Camera + ROS2 integration manual

This manual explains how to connect:

- FPV HDMI camera feeds (via USB HDMI capture cards)
- USB thermal camera
- ROS2 runtime (local or external workspace)

to the FastAPI dashboard backend and frontend.

---

## 1) Target runtime architecture

Frontend (browser UI)
↕
FastAPI backend (`backend.app`)
↕
WebSocket manager + REST API
↕
rosbridge websocket (`ws://127.0.0.1:9090` by default)
↕
ROS2 graph

Critical emergency commands bypass rosbridge:

Frontend → FastAPI → direct `rclpy` publishers → critical ROS topics.

---

## 2) Hardware connections

### A. FPV HDMI pipeline (CAM_01..CAM_04)

Connect each stream as:

FPV camera → VTX → receiver (Foxeer Lollipop 4 Plus) → HDMI output → USB HDMI capture card → robot computer USB.

Important:

- Do not use laptop HDMI input assumptions.
- Each HDMI source must appear as Linux video capture (`/dev/video*`).

### B. Thermal camera pipeline

Connect thermal camera directly over USB.

Important:

- Device path is dynamic (`/dev/video*` can change).
- Identification is by device name/class, not fixed index.
- Rotation/flip correction is controlled in software.

---

## 3) Linux camera validation

Run these before launching dashboard:

```bash
ls /dev/video*
v4l2-ctl --list-devices
```

Expected:

- HDMI capture cards listed as UVC/video capture devices.
- Thermal camera listed with vendor-specific name (thermal/IR/FLIR/Seek-like label).

If names are unstable, add udev rules (recommended for production).

### Identify exactly which `/dev/video*` is your USB camera

Use this sequence:

```bash
v4l2-ctl --list-devices
```

Then map each listed camera node to USB bus path/vendor info:

```bash
for d in /dev/video*; do
   echo "===== $d ====="
   udevadm info -q property -n "$d" | grep -E 'ID_MODEL=|ID_VENDOR_ID=|ID_MODEL_ID=|ID_PATH='
done
```

Quick visual check of the chosen node:

```bash
ffplay -f v4l2 -i /dev/video0
```

Replace `/dev/video0` with the node you want.

---

## 4) ROS2 workspace hookup (local or external)

You can use any built ROS2 workspace location.

### Option A: one-off shell startup

```bash
source /opt/ros/jazzy/setup.bash
source /absolute/path/to/your/ws/install/setup.bash
python3 start_backend.py
```

(On Ubuntu 22.04 use `/opt/ros/humble/setup.bash`; `start_backend.py` picks Jazzy first and falls
back to Humble automatically.)

### Option B: persistent launcher env vars

```bash
export ROS_DISTRO_SETUP=/opt/ros/jazzy/setup.bash
export WS_SETUP=/absolute/path/to/your/ws/install/setup.bash
python3 start_backend.py
```

`start_backend.py` starts:

1. rosbridge websocket launch
2. backend API (`uvicorn backend.app:app`)

and shuts both down on Ctrl+C.

### Camera bridge workflow (hardware)

Build and launch the ROS packages from the `robot_backend_ws` workspace, then let the dashboard consume ROS topics:

```bash
cd /home/mathinonthemoon/Documents/Comp_dah/robot_backend_ws
colcon build --packages-select dashboard_bridge camera_bridge aegis_description aegis_robot_bridge
source install/setup.bash
ros2 launch dashboard_bridge dashboard_bridge.launch.py
```

### Gazebo Classic simulation workflow

> Gazebo Classic 11 is Ubuntu 22.04/Humble only. On the `ubuntu-24-jazzy` branch the simulator
> packages are `COLCON_IGNORE`d; run this section from `main` on a 22.04 machine.

Use the simulator when no physical camera or lidar is connected. The simulation and hardware paths
can coexist, but do not run both camera publishers on the same topics at the same time.

```bash
# Terminal 1
python3 start_backend.py

# Terminal 2
source /opt/ros/jazzy/setup.bash
source robot_backend_ws/install/setup.bash
ros2 launch aegis_sim sim.launch.py gui:=true rviz:=false
```

What the simulation publishes:

| Dashboard use | Simulated source | Adapter output |
|---|---|---|
| Drive | `/cmd_vel` | `/cmd_vel_sim` to Gazebo diff drive |
| Odometry | Gazebo diff drive | `/hardware/odometry` |
| IMU | Gazebo IMU sensor | `/hardware/imu` |
| Lidar | Gazebo ray sensor | `/scan` (frontend subscribes directly) |
| Front camera | Gazebo camera | `/camera/image_compressed`, `/camera/video2/image_compressed` |
| Rear camera | Gazebo camera | `/camera/webcam/image_compressed` |
| Camera health | adapter | `/camera/video2/healthy`, `/camera/webcam/healthy`, `/camera/primary_source` |
| Proximity | adapter from `/scan` | `/proximity/range` |
| Thermal | synthetic adapter value | `/thermal/temperature` |
| Diagnostics | adapter | `/diagnostics` |
| E-stop/reset | adapter services | `/emergency/estop`, `/emergency/reset` |

The launch file does not start rosbridge; `start_backend.py` owns that process. If the dashboard is
connected but the sim is not, the existing MJPEG hardware camera path remains active. CAM_03 and
CAM_04 intentionally show `NO SIGNAL` in simulation because the Aegis model has only front/rear
cameras. See the dashboard **Lidar** tab for the live `/scan` polar plot.

Published topics used by dashboard integration:

The camera image topics are now published by `camera_bridge`. The `dashboard_bridge` package keeps the status and control nodes.

- `/camera/image_compressed`
- `/camera/video2/image_compressed`
- `/camera/webcam/image_compressed`
- `/camera/thermal/image_compressed`
- `/camera/video2/healthy`
- `/camera/webcam/healthy`
- `/camera/thermal/healthy`
- `/camera/primary_source`

The FastAPI backend also exposes current camera bridge status:

- `GET /api/v1/cameras/status`

---

## 5) Backend startup and health checks

From project root:

```bash
python3 start_backend.py
```

Open dashboard:

```text
http://127.0.0.1:8000/
```

Quick checks:

```bash
curl -I http://127.0.0.1:8000/
curl http://127.0.0.1:8000/api/v1/diagnostics
curl http://127.0.0.1:8000/api/v1/cameras/discover
```

If diagnostics works but cameras are blank, focus on camera device mapping.

---

## 6) Camera assignment workflow (Lab_View)

1. Open dashboard and click `Lab_View` in top navbar.
2. Click `Detect devices`.
3. Verify:
   - HDMI capture devices are shown in discovered list.
   - Thermal device is shown separately.
4. Assign HDMI devices to `CAM_01..CAM_04` using camera API (or Lab_View control actions).
5. Set thermal source to detected thermal device.
6. Apply thermal rotate/flip until image orientation is correct.
7. Restart specific stream if feed drops.

Relevant API endpoints:

- `GET /api/v1/cameras/discover`
- `POST /api/v1/cameras/assign?tile=CAM_01&device=/dev/video2`
- `POST /api/v1/cameras/assign?tile=THERMAL&device=/dev/video6`
- `POST /api/v1/cameras/rotate?tile=THERMAL&degrees=90`
- `POST /api/v1/cameras/flip?tile=THERMAL&horizontal=true`
- `POST /api/v1/cameras/THERMAL/restart`

Streams:

- `/api/v1/cameras/CAM_01/stream.mjpeg`
- `/api/v1/cameras/CAM_02/stream.mjpeg`
- `/api/v1/cameras/CAM_03/stream.mjpeg`
- `/api/v1/cameras/CAM_04/stream.mjpeg`
- `/api/v1/cameras/THERMAL/stream.mjpeg`
- `/api/v1/cameras/usb/stream.mjpeg?device=/dev/video0`

### Show one USB camera directly in dashboard

If you only connected one USB camera and want to show it immediately, set image source to:

```text
/api/v1/cameras/usb/stream.mjpeg?device=/dev/video0
```

For the current frontend codebase, you can also test quickly from browser:

```text
http://127.0.0.1:8000/api/v1/cameras/usb/stream.mjpeg?device=/dev/video0
```

If video appears there, use the same URL as the dashboard `<img src="...">` value.

---

## 7) ROS graph and engineering controls (Lab_View)

Use Lab_View for runtime engineering:

- ROS graph refresh
- topic echo requests
- node restart / enable / disable
- parameter edit bridge calls
- hardware diagnostics snapshot

Core endpoints:

- `GET /api/v1/labview/ros-graph`
- `POST /api/v1/labview/topic-echo?topic=/your/topic`
- `POST /api/v1/labview/node/restart?node=dashboard_bridge`
- `POST /api/v1/labview/node/disable?node=dashboard_bridge`
- `POST /api/v1/labview/node/enable?node=dashboard_bridge`
- `POST /api/v1/labview/parameter-edit?node=...&parameter=...&value=...`
- `GET /api/v1/labview/hardware`

---

## 8) Mission control and emergency path

Mission buttons map to:

- `POST /api/v1/mission/start`
- `POST /api/v1/mission/reset`
- `POST /api/v1/mission/estop`
- `POST /api/v1/mission/dpad/{direction}`

E-STOP behavior:

- Calls direct `rclpy` critical clients (`std_srvs/Trigger` services) plus a zero `Twist` motor halt on `/cmd_vel`.
- Does not rely solely on rosbridge availability.

---

## 9) WebSocket channels used by frontend

- `/ws/logs`
- `/ws/telemetry`
- `/ws/state`
- `/ws/ros-graph`
- `/ws/cameras`

No polling required for logs/state updates.

---

## 10) Production hardening checklist

- Use fixed USB ports and label each capture dongle physically.
- Add udev rules for stable symbolic names (`/dev/cam_front`, etc.).
- Disable USB autosuspend for capture devices if dropout occurs.
- Pin backend and OpenCV versions used in validation.
- Run backend with process supervisor (systemd) for auto-restart.
- Keep rosbridge and backend logs persisted to disk.
- Validate E-STOP topic wiring on every boot.

---

## 11) Troubleshooting

### Dashboard opens, no camera video

- Check `v4l2-ctl --list-devices`.
- Re-run camera discovery endpoint.
- Verify assignment points to real `/dev/video*`.
- Restart stream endpoint per tile.

### Camera shown but high latency

- Lower resolution/fps via timing endpoint.
- Ensure capture dongle supports requested mode.
- Avoid USB hubs for high-bandwidth multi-camera setups.

### Thermal upside down / mirrored

- Use rotate and flip endpoints from Lab_View.
- Confirm correction persists in runtime config.

### ROS graph empty in Lab_View

- Confirm rosbridge is running on port 9090.
- Confirm `WS_SETUP` points to correct built workspace.
- Check backend log stream for `rosbridge connect_failed` events.

### E-STOP endpoint returns error

- Verify `rclpy`, `std_srvs`, and `geometry_msgs` are installed in the active environment.
- Verify ROS domain and namespace are correct for robot runtime.

---

## 12) Recommended daily startup

```bash
cd /home/mathinonthemoon/Documents/Comp_dah
python3 start_backend.py
```

`start_backend.py` sources `robot_backend_ws/install/setup.bash` by default (override with `WS_SETUP`). In a second terminal, source the workspace and start the bridge nodes (section 4, Camera bridge workflow).

Then open `http://127.0.0.1:8000/` and validate:

1. camera tiles populate
2. thermal orientation correct
3. state/log websocket updates live
4. ROS graph visible in Lab_View
5. E-STOP command path validated in safe test mode
