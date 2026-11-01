# AGENTS.md

Rescue-robot dashboard: FastAPI backend, plain static frontend, and ROS 2 bridge packages.

This branch (`ubuntu-24-jazzy`) targets **Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12**. The
Gazebo Classic simulation stays on `main` (Ubuntu 22.04 + Humble) because Gazebo Classic 11 has
no 24.04 packages.

## Layout

- `backend/` — FastAPI app. `app.py` is the composition root; `api/` holds REST routers; also `websocket/`, `ros_bridge/`, `camera_manager/`, `thermal_manager/`, `hardware_interface/`, `config/`, `diagnostics/`, `flexbe_manager/`, `labview/`, `logs/`, `mission_control/`, `state_sync/`, `debug/`, `core/`, `utils/`.
- `frontend/` — static dashboard served by FastAPI (no build step).
- `robot_backend_ws/src/` — ROS 2 packages (`dashboard_bridge`, `camera_bridge`, `aegis_description`, `aegis_gazebo_plugins`, `aegis_robot_bridge`, `aegis_sim`).
- `firmware/esp32_motor_bridge/` — ESP32 Arduino sketch for the real M3508 robot.
- `tests/` — pytest suite; `docs/` — integration manuals (`CONNECT_CAMERAS_AND_ROS.md`, `PI_ESP32_ROBOT.md`); `start_backend.py` — launcher.

## Run / build

- Use the repo `.venv`; `start_backend.py` already prefers `.venv/bin/python`. Run commands from the repo root so `backend.*` imports resolve.
- Full stack: `python3 start_backend.py` — sources the active ROS distro (`/opt/ros/jazzy/setup.bash` on 24.04, `/opt/ros/humble/setup.bash` on 22.04, `ROS_DISTRO` respected) + `robot_backend_ws/install/setup.bash`, starts rosbridge, then `uvicorn backend.app:app --host 0.0.0.0 --port 8000` (no `--reload`).
  - Overrides: `ROS_DISTRO_SETUP`, `WS_SETUP`, `BACKEND_HOST`, `BACKEND_PORT`, `BACKEND_CMD`, `ROSBRIDGE_CMD`.
- Backend only: `.venv/bin/python -m uvicorn backend.app:app --reload` (it still imports `cv2`/`numpy`, so use the venv). `python -m backend.main` is a non-reload alternative that honors `RESCUE_API_HOST`/`RESCUE_API_PORT`.
- ROS workspace: `robot_backend_ws/` is canonical; source Jazzy first, then build and source its overlay:
  ```bash
  source /opt/ros/jazzy/setup.bash
  cd robot_backend_ws
  colcon build --packages-select dashboard_bridge camera_bridge aegis_description aegis_robot_bridge
  source install/setup.bash
  ```
  `start_backend.py` sources `robot_backend_ws/install/setup.bash` by default. The repo root is *not* a colcon workspace anymore. `aegis_sim` and `aegis_gazebo_plugins` contain an empty `COLCON_IGNORE` on this branch (Gazebo Classic 11 is 22.04-only), so a plain `colcon build` skips them; delete those files on a Humble machine to build the simulator.
- Gazebo Classic 11 simulation (Humble branch only, not available on 24.04): start the backend first (`python3 start_backend.py`), then in a sourced
  shell run `ros2 launch aegis_sim sim.launch.py` (`gui:=false` for headless, `rviz:=true` for RViz; also
  `world`, `verbose`, `use_sim_time`, `spawn_{x,y,z,yaw}`, `camera_fps`).
  The sim launch must not start a second rosbridge. It adds the plugin package's `lib/` to
  `GAZEBO_PLUGIN_PATH` and starts `aegis_dashboard_adapter` (executable `dashboard_adapter`), which owns
  `/emergency/estop` and `/emergency/reset`, relays `/cmd_vel` to `/cmd_vel_sim`, and publishes
  compressed camera/health, proximity, thermal, and diagnostics topics for the dashboard.
- Hardware robot (Raspberry Pi + ESP32): `ros2 launch aegis_robot_bridge robot.launch.py port:=/dev/aegis-esp32`
  in a sourced shell. It needs `python3-serial` and membership of the `dialout` group; `port` may be empty
  for auto-detection. It streams `/cmd_vel` to the ESP32 at 20 Hz, serves `/emergency/estop|reset`, and
  publishes `/hardware/odometry`, `/emergency/status`, `/diagnostics`. Do not run it alongside
  `aegis_sim` — both claim the same topics and services.

## Architecture

- `backend/app.py` is the composition root. `build_services()` creates every service; `lifespan` wires WebSocket broadcast and starts camera threads. Add new services there and reach them via `request.app.state.services["name"]`.
- REST routers live in `backend/api/*.py` and are mounted under `/api/v1` by `backend/api/routes.py` (`/mission`, `/cameras`, `/config`, `/diagnostics`, `/labview`). WebSocket channels are declared explicitly in `app.py` (`/ws/logs`, `/ws/telemetry`, `/ws/state`, `/ws/ros-graph` + `/ws/ros_graph`, `/ws/cameras` + `/ws/camera_metadata`); unknown channels close with code 1008.
- Critical path bypasses rosbridge: E-stop/reset call `std_srvs/Trigger` **services**, motor halt publishes zero `geometry_msgs/Twist` on `/cmd_vel`, all through direct `rclpy` in `backend/hardware_interface/critical.py` (it degrades to "rclpy unavailable" results when ROS is absent). Everything else uses rosbridge (`ws://127.0.0.1:9090`), including `backend/ros_bridge/camera_status.py`, which tracks `/camera/{video2,webcam}/healthy` and `/camera/primary_source`.
- Settings use pydantic-settings with prefix `RESCUE_` and `.env` (`backend/core/settings.py`), e.g. `RESCUE_ROSBRIDGE_URL`, `RESCUE_RUNTIME_CONFIG_PATH`, `RESCUE_CRITICAL_ESTOP_SERVICE`. The default runtime config path is absolute (`backend/config/runtime.json`), so the backend no longer depends on cwd for config.
- `frontend/` is plain HTML/CSS/JS with no build step; FastAPI serves it. Edit only `frontend/` — root `index.html`/`style.css` were removed. Only `/` and existing files are served; other paths return 404 (no SPA fallback). Scripts load in order: `ros-config.js` (topic/service table, rosbridge URLs derived from `window.location`), `ros-connection.js` (`RosBridgeConnection`: reconnect + subscriptions), `ros-store.js` (state store and message parsers), `script.js` (DOM wiring/rendering). The frontend opens its own rosbridge connection alongside backend REST/WebSockets.
- ROS packages: `dashboard_bridge` (all hardware node entrypoints in `dashboard_bridge/nodes.py` + `dashboard_bridge.launch.py`), `camera_bridge` (`camera_bridge/node.py`, device discovery by name, not fixed index), `aegis_description` (URDF/xacro shared by RViz and Gazebo Classic), `aegis_gazebo_plugins` (C++ `flipper_controller` Gazebo plugin driven by `/flipper_cmd` + `/cmd_vel_sim`), `aegis_robot_bridge` (Pi-side serial bridge: `protocol.py` codec + kinematics, `serial_link.py` pyserial threads, `node.py` ROS layer), and `aegis_sim` (world, `spawn_model`, `dashboard_adapter`, `math_helpers`).
- The hardware drive path is serial, not rosbridge: the browser publishes `/cmd_vel` through rosbridge, `aegis_robot_bridge` converts it to per-side RPM (`twist_to_wheel_rpm`) and streams `VEL,<l>,<r>` at 20 Hz; the sketch acknowledges `STOP`/`ESTOP`/`RESET` and reports `TEL` telemetry. Both ends zero the wheels on link loss (`command_hold_s` on the Pi, 400 ms firmware watchdog), and the firmware E-stop latch survives until `RESET`.
- Camera bridge publishes `/camera/{video2,webcam,thermal}/image_compressed` and `/healthy`, plus `/camera/image_compressed` and `/camera/primary_source`; the backend has its own `/dev/video*` capture path too. The sim adapter uses raw Gazebo camera topics and compresses them with `cv2` (no `cv_bridge` runtime dependency).

## Verify / gotchas

- Lint: `.venv/bin/ruff check .` (config in `pyproject.toml`: line length 140, rules `E,F,I,B`). Tests: `.venv/bin/pytest` or a single test with `.venv/bin/pytest tests/test_config_service.py::test_config_service_creates_defaults_and_deep_merges` (see `tests/`). CI runs both via `.github/workflows/checks.yml`.
- Python is 3.12 on this branch (CI runs 3.12, ruff `target-version = "py312"`). Recreate `.venv` with `python3 -m venv .venv` after moving a checkout between 22.04 and 24.04 — a venv from the other distro will not run.
- Tests run with no ROS and no `cv2`: `pyproject.toml` puts `robot_backend_ws/src/aegis_sim` and `robot_backend_ws/src/aegis_robot_bridge` on `pythonpath` and disables the system `launch_testing`/`launch_ros` plugins. Prefer testing pure helpers (`backend/config`, `aegis_sim/math_helpers.py`, `aegis_robot_bridge/protocol.py`) over importing `backend.app` or ROS nodes.
- `.venv/` is not tracked (`.gitignore`); do not re-add it or commit `__pycache__`/`build`/`install`/`log` output. `backend/config/runtime.json` is runtime state written by `ConfigService` and is ignored.
- Manual smoke test: start the backend, then `curl http://127.0.0.1:8000/api/v1/diagnostics` and `/api/v1/cameras/discover`. Testing the E-STOP path requires a live `rclpy` environment and `/emergency/estop` service (`POST /api/v1/mission/estop`, reset via `/api/v1/mission/reset`).
- Sim smoke test (Humble branch): `ros2 launch aegis_sim sim.launch.py gui:=false`; check `/scan` (~12 Hz, 720 samples, 0.12–12 m), `/hardware/odometry` (~50 Hz), `/camera/video2/image_compressed`, and call `/emergency/estop` + `/emergency/reset`. Sim package tests: `cd robot_backend_ws && colcon test --packages-select aegis_sim`. URDF: expand the xacro and run `check_urdf`.
- Bridge smoke test without hardware: run the node against a pty (`pty.openpty()`, feed `TEL,...` lines, answer `ESTOP`/`RESET` with `OK,...`) on an isolated `ROS_DOMAIN_ID`; check that `/hardware/odometry` advances, that `VEL` is zeroed after `command_hold_s`, and that `/emergency/estop` returns `success: true`.
- `requirements.txt` covers only Python web/camera deps; `requirements-dev.txt` is the minimal lint/test subset. `rclpy`, `rosbridge_server`, `gazebo_ros`, and ROS message packages must come from the system ROS Humble environment.
- `.agents/skills/` holds installed agent skills tracked by `skills-lock.json` (`impeccable` for frontend design work, `urdf`, `robotics-testing`, `safe-debug`).
