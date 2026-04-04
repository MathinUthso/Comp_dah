from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ROS_DISTRO_SETUP = os.environ.get("ROS_DISTRO_SETUP", "/opt/ros/humble/setup.bash")
WS_SETUP = os.environ.get("WS_SETUP", str(ROOT / "robot_backend_ws" / "install" / "setup.bash"))
BACKEND_HOST = os.environ.get("BACKEND_HOST", "0.0.0.0")
BACKEND_PORT = os.environ.get("BACKEND_PORT", "8000")
BACKEND_CMD = os.environ.get("BACKEND_CMD", f"python3 -m uvicorn backend.app:app --host {BACKEND_HOST} --port {BACKEND_PORT} --reload")
ROSBRIDGE_CMD = os.environ.get("ROSBRIDGE_CMD", "ros2 launch rosbridge_server rosbridge_websocket_launch.xml")


def launch(command: str) -> subprocess.Popen:
    shell = f"set -e; source '{ROS_DISTRO_SETUP}'; source '{WS_SETUP}' 2>/dev/null || true; {command}"
    return subprocess.Popen(["bash", "-lc", shell], cwd=str(ROOT))


def main() -> int:
    processes: list[subprocess.Popen] = []

    try:
        print(f"[launcher] sourcing {ROS_DISTRO_SETUP}")
        print(f"[launcher] sourcing {WS_SETUP}")
        print("[launcher] starting rosbridge")
        processes.append(launch(ROSBRIDGE_CMD))
        time.sleep(1.5)

        print("[launcher] starting backend")
        processes.append(launch(BACKEND_CMD))

        print("[launcher] dashboard online at http://127.0.0.1:8000/")
        print("[launcher] press Ctrl+C to stop")

        while True:
            alive = [p.poll() for p in processes]
            if any(code not in (None, 0) for code in alive):
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[launcher] shutting down...")
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                try:
                    process.send_signal(signal.SIGINT)
                except Exception:
                    pass
        deadline = time.time() + 10
        for process in reversed(processes):
            while process.poll() is None and time.time() < deadline:
                time.sleep(0.1)
            if process.poll() is None:
                try:
                    process.kill()
                except Exception:
                    pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
