# Rescue robot dashboard backend

## Architecture

### 1. FastAPI
FastAPI handles operator-facing REST APIs, WebSocket fan-out, camera routing, config management, Lab_View engineering tools, diagnostics aggregation, and hardware-control orchestration.

### 2. rosbridge
rosbridge provides dynamic ROS graph inspection, topic/service access, and non-critical pub/sub integration for telemetry, FlexBE introspection, and engineering tools.

### 3. Direct ROS2 node communication
Direct `rclpy` publishers are reserved for critical paths: E-STOP, motor halt, and emergency reset.

## Folder structure

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

robot_backend_ws/
└── src/
    ├── dashboard_bridge/
    ├── camera_bridge/
    ├── thermal_bridge/
    ├── hardware_control/
    ├── flexbe_adapter/
    ├── diagnostics_node/
    └── emergency_control/

## Deployment steps

1. Install Python dependencies from requirements.txt.
2. Build the ROS2 workspace in robot_backend_ws with colcon.
3. Source ROS Humble and the workspace overlay.
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

## Integration manual

See docs/CONNECT_CAMERAS_AND_ROS.md for full camera wiring, ROS workspace connection, startup, Lab_View operations, and troubleshooting steps.
