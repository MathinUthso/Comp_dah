"""Threaded pyserial transport for the ESP32 drive link.

The reader and writer run in their own threads so a wedged USB adapter can
never block the ROS executor: callbacks only push strings into a queue and the
latest decoded lines are polled from :meth:`SerialLink.drain`.
"""

from __future__ import annotations

import glob
import threading
import time
from collections import deque

BY_ID_GLOB = "/dev/serial/by-id/*"
DEVICE_GLOBS = ("/dev/ttyUSB*", "/dev/ttyACM*")
# Names reported by /dev/serial/by-id for the adapters commonly used on ESP32 boards.
PREFERRED_HINTS = ("esp32", "esp32s3", "cp210", "ch340", "ch910", "ftdi", "usb-serial", "usb_serial")


def pick_port(candidates: list[str], preferred_hints: tuple[str, ...] = PREFERRED_HINTS) -> str | None:
    """Pick a serial device from candidates, preferring known ESP32 adapters."""
    ordered = sorted(candidates)
    for path in ordered:
        lowered = path.lower()
        if any(hint in lowered for hint in preferred_hints):
            return path
    return ordered[0] if ordered else None


def resolve_port() -> str | None:
    """Resolve a default port from /dev/serial/by-id, then ttyUSB/ttyACM."""
    by_id = glob.glob(BY_ID_GLOB)
    chosen = pick_port(by_id)
    if chosen:
        return chosen
    devices: list[str] = []
    for pattern in DEVICE_GLOBS:
        devices.extend(glob.glob(pattern))
    return pick_port(devices)


class SerialLink:
    """Reconnecting, non-blocking line transport."""

    def __init__(self, port: str = "", baud: int = 115200, reconnect_interval_s: float = 1.0, max_lines: int = 64) -> None:
        self._port_arg = port.strip()
        self._baud = int(baud)
        self._reconnect_interval_s = max(0.2, float(reconnect_interval_s))
        self._lines: deque[str] = deque(maxlen=max_lines)
        self._tx: deque[str] = deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._serial: object | None = None
        self._resolved_port = ""
        self._threads: list[threading.Thread] = []
        self.last_rx_monotonic = 0.0
        self.last_error = ""
        self.tx_count = 0
        self.rx_count = 0

    @property
    def connected(self) -> bool:
        return self._serial is not None

    @property
    def port(self) -> str:
        return self._resolved_port or self._port_arg

    @property
    def baud(self) -> int:
        return self._baud

    def start(self) -> None:
        if self._threads:
            return
        self._stop.clear()
        for name, target in (("aegis-serial-reader", self._read_loop), ("aegis-serial-writer", self._write_loop)):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self) -> None:
        self._stop.set()
        self._close()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()

    def send(self, line: str) -> bool:
        """Queue one newline-framed command. Returns False when the link is down."""
        if not self.connected:
            return False
        with self._lock:
            self._tx.append(line)
        return True

    def drain(self) -> list[str]:
        """Return and clear all received lines since the last call."""
        with self._lock:
            lines = list(self._lines)
            self._lines.clear()
        return lines

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------
    def _read_loop(self) -> None:
        while not self._stop.is_set():
            if self._serial is None:
                if not self._open():
                    self._stop.wait(self._reconnect_interval_s)
                    continue
            try:
                raw = self._serial.readline()  # type: ignore[attr-defined]
            except Exception as exc:  # device unplugged, driver reset, ...
                self._fail(str(exc))
                continue
            if not raw:
                continue
            try:
                text = raw.decode("ascii", "ignore").strip()
            except AttributeError:
                continue
            if not text:
                continue
            with self._lock:
                self._lines.append(text)
            self.rx_count += 1
            self.last_rx_monotonic = time.monotonic()

    def _write_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                line = self._tx.popleft() if self._tx else ""
            if not line:
                self._stop.wait(0.02)
                continue
            serial_port = self._serial
            if serial_port is None:
                continue  # drop commands queued for a link that went away
            try:
                serial_port.write((line + "\n").encode("ascii", "ignore"))  # type: ignore[attr-defined]
                serial_port.flush()  # type: ignore[attr-defined]
            except Exception as exc:
                self._fail(str(exc))
                continue
            self.tx_count += 1

    # ------------------------------------------------------------------
    # Connection handling
    # ------------------------------------------------------------------
    def _open(self) -> bool:
        port = self._port_arg or resolve_port()
        if not port:
            self.last_error = "no serial device found"
            return False
        try:
            import serial  # imported lazily so the package imports without pyserial
        except Exception as exc:  # pragma: no cover - depends on host install
            self.last_error = f"pyserial unavailable: {exc}"
            return False
        try:
            self._serial = serial.Serial(port, self._baud, timeout=0.2, write_timeout=0.2)
        except Exception as exc:
            self.last_error = str(exc)
            self._serial = None
            return False
        self._resolved_port = port
        self.last_error = ""
        return True

    def _close(self) -> None:
        serial_port = self._serial
        self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()  # type: ignore[attr-defined]
            except Exception:
                pass

    def _fail(self, message: str) -> None:
        self.last_error = message
        self._close()
        self._stop.wait(0.2)
