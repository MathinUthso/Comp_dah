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

---

## 4) ROS2 workspace hookup (local or external)

You can use any built ROS2 workspace location.

### Option A: one-off shell startup

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/your/ws/install/setup.bash
python3 start_backend.py
```

### Option B: persistent launcher env vars

```bash
export ROS_DISTRO_SETUP=/opt/ros/humble/setup.bash
export WS_SETUP=/absolute/path/to/your/ws/install/setup.bash
python3 start_backend.py
```

`start_backend.py` starts:

1. rosbridge websocket launch
2. backend API (`uvicorn backend.app:app`)

and shuts both down on Ctrl+C.

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

- Calls direct `rclpy` critical publishers (estop + motor halt).
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

- Verify `rclpy` and `std_msgs` installed in active environment.
- Verify ROS domain and namespace are correct for robot runtime.

---

## 12) Recommended daily startup

```bash
cd /home/mathinonthemoon/Documents/Comp_dah
export WS_SETUP=/absolute/path/to/your/ws/install/setup.bash
python3 start_backend.py
```

Then open `http://127.0.0.1:8000/` and validate:

1. camera tiles populate
2. thermal orientation correct
3. state/log websocket updates live
4. ROS graph visible in Lab_View
5. E-STOP command path validated in safe test mode
