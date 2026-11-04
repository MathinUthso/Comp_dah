#!/usr/bin/env bash
# Quick post-start check for the dashboard, ROS graph and Pi drive topics.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BASE_URL="${BASE_URL:-http://127.0.0.1:8000}"
FAIL=0

if [[ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi
if [[ -f "$ROOT/robot_backend_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/robot_backend_ws/install/setup.bash"
fi

check_http() {
  local path="$1"
  if curl -fsS --max-time 5 "$BASE_URL$path" >/dev/null; then
    echo "OK   GET $path"
  else
    echo "FAIL GET $path"
    FAIL=1
  fi
}

check_http /api/v1/diagnostics
check_http /api/v1/cameras/discover

if command -v ros2 >/dev/null 2>&1; then
  topics="$(ros2 topic list 2>/dev/null || true)"
  for topic in /cmd_vel /diagnostics /hardware/odometry /emergency/status; do
    if grep -qx "$topic" <<<"$topics"; then
      echo "OK   topic $topic"
    else
      echo "FAIL topic $topic"
      FAIL=1
    fi
  done

  services="$(ros2 service list 2>/dev/null || true)"
  for service in /emergency/estop /emergency/reset; do
    if grep -qx "$service" <<<"$services"; then
      echo "OK   service $service"
    else
      echo "FAIL service $service"
      FAIL=1
    fi
  done

  for topic in /camera/primary_source /camera/video2/image_compressed; do
    if grep -qx "$topic" <<<"$topics"; then
      echo "OK   topic $topic"
    else
      echo "WARN topic $topic not advertised"
    fi
  done
else
  echo "WARN ros2 not on PATH; skipped ROS graph checks"
fi

if [[ -e /dev/aegis-esp32 ]]; then
  echo "OK   serial /dev/aegis-esp32"
else
  echo "WARN /dev/aegis-esp32 missing; the bridge will auto-detect the ESP32"
fi

exit "$FAIL"
