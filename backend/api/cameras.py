from __future__ import annotations

import time

import cv2
import numpy as np
from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse

try:
    import mediapipe as mp
except Exception:  # pragma: no cover - optional dependency fallback
    mp = None

router = APIRouter()
STREAM_FRAME_WIDTH = 180
STREAM_FRAME_HEIGHT = 80
STREAM_FRAME_SIZE = (STREAM_FRAME_WIDTH, STREAM_FRAME_HEIGHT)
JPEG_QUALITY = 80
_HAND_GESTURE_DETECTOR = None


def _mediapipe_hands_available():
    return mp is not None and hasattr(mp, "solutions") and hasattr(mp.solutions, "hands")


def _normalize_frame(frame):
    src_h, src_w = frame.shape[:2]
    if src_h <= 0 or src_w <= 0:
        return np.zeros((STREAM_FRAME_HEIGHT, STREAM_FRAME_WIDTH, 3), dtype=np.uint8)
    scale = min(STREAM_FRAME_WIDTH / src_w, STREAM_FRAME_HEIGHT / src_h)
    resized_w = max(1, int(src_w * scale))
    resized_h = max(1, int(src_h * scale))
    resized = cv2.resize(frame, (resized_w, resized_h), interpolation=cv2.INTER_AREA)
    canvas = cv2.copyMakeBorder(
        resized,
        (STREAM_FRAME_HEIGHT - resized_h) // 2,
        STREAM_FRAME_HEIGHT - resized_h - (STREAM_FRAME_HEIGHT - resized_h) // 2,
        (STREAM_FRAME_WIDTH - resized_w) // 2,
        STREAM_FRAME_WIDTH - resized_w - (STREAM_FRAME_WIDTH - resized_w) // 2,
        cv2.BORDER_CONSTANT,
        value=(0, 0, 0),
    )
    return canvas


def _fit_frame_to_stream(frame):
    if frame is None:
        return np.zeros((STREAM_FRAME_HEIGHT, STREAM_FRAME_WIDTH, 3), dtype=np.uint8)
    return _normalize_frame(frame)


def _fallback_frame(text: str):
    frame = np.zeros((STREAM_FRAME_HEIGHT, STREAM_FRAME_WIDTH, 3), dtype=np.uint8)
    cv2.putText(frame, "CAMERA UNAVAILABLE", (130, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, text, (40, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(frame, "Waiting for ROS camera bridge...", (140, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 255, 120), 1, cv2.LINE_AA)
    return frame


def _mjpeg_chunk(frame):
    normalized = _fit_frame_to_stream(frame)
    ok, encoded = cv2.imencode(".jpg", normalized, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        return None
    return b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(encoded)).encode() + b"\r\n\r\n" + encoded.tobytes() + b"\r\n"


def _contour_fallback_box(frame):
    """Fallback detector used only when MediaPipe is unavailable."""
    if frame is None:
        return None
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        h, w = gray.shape[:2]
        if h <= 0 or w <= 0:
            return None
        if area < float(h * w) * 0.01:
            return None
        x, y, bw, bh = cv2.boundingRect(contour)
        return {
            "x": x / w,
            "y": y / h,
            "w": bw / w,
            "h": bh / h,
            "confidence": min(1.0, area / float(h * w)),
            "gesture": "OBJECT",
            "model": "contour-fallback",
        }
    except Exception:
        return None


class _HandGestureDetector:
    """MediaPipe-based single-hand detector with simple gesture classification."""

    def __init__(self):
        if not _mediapipe_hands_available():
            raise RuntimeError("mediapipe hands module unavailable")
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    @staticmethod
    def _fingers_up(landmarks, handedness_label: str):
        tip_ids = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
        pip_ids = {"thumb": 3, "index": 6, "middle": 10, "ring": 14, "pinky": 18}

        if handedness_label == "Right":
            thumb_up = landmarks[tip_ids["thumb"]].x > landmarks[pip_ids["thumb"]].x
        else:
            thumb_up = landmarks[tip_ids["thumb"]].x < landmarks[pip_ids["thumb"]].x

        return {
            "thumb": thumb_up,
            "index": landmarks[tip_ids["index"]].y < landmarks[pip_ids["index"]].y,
            "middle": landmarks[tip_ids["middle"]].y < landmarks[pip_ids["middle"]].y,
            "ring": landmarks[tip_ids["ring"]].y < landmarks[pip_ids["ring"]].y,
            "pinky": landmarks[tip_ids["pinky"]].y < landmarks[pip_ids["pinky"]].y,
        }

    @staticmethod
    def _classify_gesture(landmarks, fingers_up):
        count = sum(1 for value in fingers_up.values() if value)
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        thumb_index_dist = float(np.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y))

        if count == 0:
            return "FIST"
        if count == 5:
            return "OPEN_PALM"
        if fingers_up["index"] and not fingers_up["middle"] and not fingers_up["ring"] and not fingers_up["pinky"]:
            return "POINT"
        if fingers_up["index"] and fingers_up["middle"] and not fingers_up["ring"] and not fingers_up["pinky"]:
            return "V_SIGN"
        if fingers_up["thumb"] and fingers_up["index"] and not fingers_up["middle"] and not fingers_up["ring"] and not fingers_up["pinky"] and thumb_index_dist < 0.06:
            return "OK"
        return f"HAND_{count}"

    def detect(self, frame):
        if frame is None:
            return None
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self._hands.process(rgb)
        if not result.multi_hand_landmarks:
            return None

        hand_landmarks = result.multi_hand_landmarks[0]
        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return None

        xs = [max(0.0, min(1.0, landmark.x)) for landmark in hand_landmarks.landmark]
        ys = [max(0.0, min(1.0, landmark.y)) for landmark in hand_landmarks.landmark]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        pad = 0.03
        min_x = max(0.0, min_x - pad)
        min_y = max(0.0, min_y - pad)
        max_x = min(1.0, max_x + pad)
        max_y = min(1.0, max_y + pad)

        handedness = result.multi_handedness[0].classification[0] if result.multi_handedness else None
        handedness_label = handedness.label if handedness else "Unknown"
        confidence = float(handedness.score) if handedness else 0.5

        fingers_up = self._fingers_up(hand_landmarks.landmark, handedness_label)
        gesture = self._classify_gesture(hand_landmarks.landmark, fingers_up)

        return {
            "x": min_x,
            "y": min_y,
            "w": max(0.0, max_x - min_x),
            "h": max(0.0, max_y - min_y),
            "confidence": confidence,
            "gesture": gesture,
            "handedness": handedness_label,
            "model": "mediapipe-hands",
        }


def _get_hand_gesture_detector():
    global _HAND_GESTURE_DETECTOR
    if not _mediapipe_hands_available():
        return None
    if _HAND_GESTURE_DETECTOR is None:
        try:
            _HAND_GESTURE_DETECTOR = _HandGestureDetector()
        except Exception:
            _HAND_GESTURE_DETECTOR = None
    return _HAND_GESTURE_DETECTOR


def _detect_hand_gesture(frame):
    detector = _get_hand_gesture_detector()
    try:
        if detector is None:
            return _contour_fallback_box(frame)
        detected = detector.detect(frame)
        if detected is not None:
            return detected
        return _contour_fallback_box(frame)
    except Exception:
        return _contour_fallback_box(frame)


def _draw_detection_overlay(frame, box):
    if frame is None or not isinstance(box, dict):
        return frame

    height, width = frame.shape[:2]
    if height <= 0 or width <= 0:
        return frame

    x = int(max(0.0, min(1.0, float(box.get("x", 0.0)))) * width)
    y = int(max(0.0, min(1.0, float(box.get("y", 0.0)))) * height)
    box_width = int(max(0.0, min(1.0, float(box.get("w", 0.0)))) * width)
    box_height = int(max(0.0, min(1.0, float(box.get("h", 0.0)))) * height)

    if box_width <= 0 or box_height <= 0:
        return frame

    x2 = min(width - 1, x + box_width)
    y2 = min(height - 1, y + box_height)
    label = str(box.get("gesture") or box.get("model") or "DETECTED")
    confidence = box.get("confidence")
    if isinstance(confidence, (int, float)):
        label = f"{label} {confidence * 100:.0f}%"

    cv2.rectangle(frame, (x, y), (x2, y2), (0, 229, 255), 2)

    label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    label_top = max(18, y - 8)
    label_left = min(max(0, x), max(0, width - label_size[0] - 8))
    cv2.rectangle(
        frame,
        (label_left, label_top - label_size[1] - baseline - 4),
        (label_left + label_size[0] + 8, label_top + baseline),
        (0, 100, 200),
        -1,
    )
    cv2.putText(frame, label, (label_left + 4, label_top - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 229, 255), 1, cv2.LINE_AA)
    return frame


def _svc(request: Request):
    return request.app.state.services["labview"]


def _camera_manager(request: Request):
    return request.app.state.services["camera_manager"]


@router.get("/discover")
async def discover(request: Request):
    return await _svc(request).detect_devices()


@router.get("/metadata")
async def metadata(request: Request):
    return await _svc(request).stream_diagnostics()


@router.get("/status")
async def camera_status(request: Request):
    return request.app.state.services["camera_status"].snapshot()


@router.post("/assign")
async def assign_source(request: Request, tile: str, device: str):
    return await _svc(request).assign_source(tile, device)


@router.post("/rotate")
async def rotate_source(request: Request, tile: str, degrees: int):
    return await _svc(request).rotate_selected_source(tile, degrees)


@router.post("/flip")
async def flip_source(request: Request, tile: str, horizontal: bool = False, vertical: bool = False):
    return await _svc(request).flip_selected_source(tile, horizontal=horizontal, vertical=vertical)


@router.post("/timing")
async def timing(request: Request, tile: str, fps: int, width: int, height: int):
    return await _svc(request).change_camera_timing(tile, fps, width, height)


@router.post("/{tile}/restart")
async def restart_stream(tile: str, request: Request):
    return await _svc(request).restart_stream(tile)


@router.get("/usb/stream.mjpeg")
async def usb_mjpeg_stream(device: str = "/dev/video0"):
    capture_target = int(device) if isinstance(device, str) and device.isdigit() else device

    def frame_generator():
        capture = cv2.VideoCapture(capture_target)
        if not capture.isOpened():
            # Keep streaming valid JPEG frames so browser <img> remains visible.
            while True:
                frame = _fallback_frame(f"Source: {device}")
                chunk = _mjpeg_chunk(frame)
                if chunk is not None:
                    yield chunk
                time.sleep(0.25)
            return
        
        try:
            while True:
                ret, frame = capture.read()
                if not ret or frame is None:
                    fallback = _fallback_frame(f"Read failed: {device}")
                    chunk = _mjpeg_chunk(fallback)
                    if chunk is not None:
                        yield chunk
                    time.sleep(0.1)
                    continue

                chunk = _mjpeg_chunk(frame)
                if chunk is None:
                    time.sleep(0.033)
                    continue

                yield chunk
                time.sleep(0.033)  # ~30 fps
        finally:
            capture.release()

    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")


@router.get("/{tile}/stream.mjpeg")
async def mjpeg_stream(tile: str, request: Request):
    camera_manager = _camera_manager(request)

    def frame_generator():
        while True:
            frame = camera_manager.frame_snapshot(tile)
            if frame is None:
                time.sleep(0.05)
                continue
            normalized = _fit_frame_to_stream(frame)
            detection = _detect_hand_gesture(frame)
            if detection is not None:
                normalized = _draw_detection_overlay(normalized, detection)
            ok, encoded = cv2.imencode(".jpg", normalized, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                time.sleep(0.05)
                continue
            chunk = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(encoded)).encode() + b"\r\n\r\n" + encoded.tobytes() + b"\r\n"
            yield chunk
            time.sleep(1.0 / max(camera_manager.hdmi_slots[tile].settings.fps if tile != "THERMAL" else camera_manager.thermal.settings.fps, 1))

    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")


@router.get("/{tile}/detections")
async def tile_detections(tile: str, request: Request):
    camera_manager = _camera_manager(request)
    frame = camera_manager.frame_snapshot(tile)
    box = _detect_hand_gesture(frame)
    gesture = box.get("gesture") if isinstance(box, dict) else ""
    return {
        "tile": tile,
        "detected": box is not None,
        "target_id": gesture or (f"{tile}_TARGET" if box is not None else ""),
        "box": box,
    }
