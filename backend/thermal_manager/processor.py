from __future__ import annotations

from dataclasses import dataclass

import cv2


@dataclass
class ThermalProcessor:
    rotation: int = 90
    flip_horizontal: bool = False
    flip_vertical: bool = False
    colorize: bool = False

    def process(self, frame):
        if self.rotation == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)
        if self.flip_vertical:
            frame = cv2.flip(frame, 0)
        if self.colorize and len(frame.shape) == 2:
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_INFERNO)
        return frame
