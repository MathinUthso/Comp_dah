#!/usr/bin/env bash
# Bring up rosbridge, the Pi camera/drive bridges and the FastAPI dashboard.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

robot_cmd="ros2 launch aegis_robot_bridge robot.launch.py"
if [[ -n "${AEGIS_PORT:-}" ]]; then
  robot_cmd+=" port:=${AEGIS_PORT}"
fi

if [[ "${SKIP_ROBOT:-0}" == "1" ]]; then
  export ROBOT_BRIDGE_CMD=""
else
  export ROBOT_BRIDGE_CMD="${ROBOT_BRIDGE_CMD:-$robot_cmd}"
fi

if [[ "${SKIP_CAMERA:-0}" == "1" ]]; then
  export CAMERA_BRIDGE_CMD=""
else
  export CAMERA_BRIDGE_CMD="${CAMERA_BRIDGE_CMD:-ros2 launch camera_bridge camera_bridge.launch.py}"
fi

# camera_bridge owns /dev/video*; keep the backend's local capture path off.
export RESCUE_ENABLE_CAMERA_CAPTURE="${RESCUE_ENABLE_CAMERA_CAPTURE:-false}"

exec python3 "$ROOT/start_backend.py"
