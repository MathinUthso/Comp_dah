from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import Image

pytest.importorskip("rclpy")

from aegis_sim.dashboard_adapter import DashboardAdapter  # noqa: E402


class _Logger:
    def warning(self, *_args, **_kwargs) -> None:
        pass


class _Encoder:
    _jpeg_quality = 80

    def get_logger(self) -> _Logger:
        return _Logger()


def test_rgb8_image_is_encoded_as_jpeg() -> None:
    frame = np.zeros((24, 32, 3), dtype=np.uint8)
    frame[:, :, 0] = 255
    message = Image()
    message.height = 24
    message.width = 32
    message.encoding = "rgb8"
    message.step = 32 * 3
    message.data = frame.tobytes()

    encoded = DashboardAdapter._encode_image(_Encoder(), message)

    assert encoded is not None
    assert encoded.startswith(b"\xff\xd8")
    assert encoded.endswith(b"\xff\xd9")


def test_invalid_encoding_is_rejected() -> None:
    message = Image()
    message.height = 4
    message.width = 4
    message.encoding = "32FC1"
    message.step = 16
    message.data = b"\0" * 64

    assert DashboardAdapter._encode_image(_Encoder(), message) is None
