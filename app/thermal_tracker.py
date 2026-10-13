"""
thermal_tracker.py - OpenCV blob detection on an MLX90640 frame.

Takes the 32x24 float array (degrees C, what Adafruit_MLX90640's
getFrame() gives on the ESP32) and finds the biggest warm blob, which is
treated as the target.
"""
import numpy as np
import cv2

from .boresight import compute_boresight_error, BoresightError, MLX90640_COLS, MLX90640_ROWS

# Starting threshold. A person is usually a few degrees above ambient but
# this needs tuning with real frames, summer outdoors and the indoor
# bench are really different. Probably should be adaptive
# (mean + N*stddev) once there's real data.
DEFAULT_THRESHOLD_C = 28.0
MIN_BLOB_AREA_PX = 4  # smaller than this is noise, not a person

# ---- frame format, has to match follower-main esp32_thermal/thermal_wire.h ----
# one UDP packet per frame, 776 bytes, little endian:
#   offset 0  float32  min_temp_c
#   offset 4  float32  scale_c   (degrees C per step, >= 0)
#   offset 8  uint8[768]  q      row-major 24 x 32
#   temp_c = min_temp_c + q * scale_c
# raw float32 frames (3072 bytes) don't work, arduino-esp32's WiFiUDP sends
# a packet every 1460 bytes so each frame got split into 3.
FRAME_PIXELS = MLX90640_COLS * MLX90640_ROWS          # 768
FRAME_HEADER_BYTES = 8
FRAME_BYTES = FRAME_HEADER_BYTES + FRAME_PIXELS       # 776
_HEADER = np.dtype([("min_temp_c", "<f4"), ("scale_c", "<f4")])


def frame_from_bytes(raw: bytes) -> np.ndarray:
    """
    One UDP payload -> (24, 32) float32 array in degrees C.
    ValueError for anything that isn't exactly one valid frame.
    """
    if len(raw) != FRAME_BYTES:
        raise ValueError(f"expected {FRAME_BYTES}-byte frame, got {len(raw)} bytes")
    header = np.frombuffer(raw, dtype=_HEADER, count=1)[0]
    min_c, scale_c = float(header["min_temp_c"]), float(header["scale_c"])
    if not (np.isfinite(min_c) and np.isfinite(scale_c)) or scale_c < 0:
        raise ValueError(f"bad frame header: min={min_c} scale={scale_c}")
    q = np.frombuffer(raw, dtype=np.uint8, offset=FRAME_HEADER_BYTES)
    temps = np.float32(min_c) + q.astype(np.float32) * np.float32(scale_c)
    return temps.reshape((MLX90640_ROWS, MLX90640_COLS))


def frame_to_bytes(frame: np.ndarray) -> bytes:
    """
    Python version of the ESP32 encoder (thermal_wire.h), for tests and
    for sending the server fake frames. NaN/inf pixels -> q = 0.
    """
    f = np.asarray(frame, dtype=np.float32).reshape(-1)
    if f.size != FRAME_PIXELS:
        raise ValueError(f"expected {FRAME_PIXELS} pixels, got {f.size}")
    finite = np.isfinite(f)
    if not finite.any():
        raise ValueError("frame has no finite pixels")
    lo, hi = np.float32(f[finite].min()), np.float32(f[finite].max())
    scale = np.float32((hi - lo) / np.float32(255.0))
    q = np.zeros(FRAME_PIXELS, dtype=np.uint8)
    if scale > 0:
        # np.round rounds half to even, C lroundf rounds half away from zero.
        # values are >= 0 here so floor(x + 0.5) matches lroundf
        steps = np.floor((f[finite] - lo) / scale + np.float32(0.5))
        q[finite] = np.clip(steps, 0, 255).astype(np.uint8)
    return np.array([(lo, scale)], dtype=_HEADER).tobytes() + q.tobytes()


def find_warm_blob(frame: np.ndarray, threshold_c: float = DEFAULT_THRESHOLD_C):
    """
    (cx, cy, area_px) of the biggest warm blob, or None if nothing is
    above the threshold.
    """
    mask = (frame >= threshold_c).astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_BLOB_AREA_PX:
        return None

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return (cx, cy, int(area))


def track(frame: np.ndarray, threshold_c: float = DEFAULT_THRESHOLD_C) -> BoresightError | None:
    """Frame in, boresight error out (None if there's no target)."""
    blob = find_warm_blob(frame, threshold_c)
    if blob is None:
        return None
    cx, cy, area = blob
    return compute_boresight_error(cx, cy, area)
