"""
thermal_tracker.py — OpenCV blob detection on a raw MLX90640 frame.

Takes the raw 32x24 float array (degrees C, exactly what
Adafruit_MLX90640's getFrame() produces on the ESP32 side) and finds
the largest sufficiently-warm blob, treating it as the tracked subject.
"""
import numpy as np
import cv2

from .boresight import compute_boresight_error, BoresightError, MLX90640_COLS, MLX90640_ROWS

# Starting threshold — a warm body against a cooler background is
# usually several degrees above ambient. This WILL need tuning once you
# have real frames; outdoor summer ambient vs. indoor test bench are
# very different baselines. Consider making this adaptive (e.g.
# mean + N*stddev) rather than a fixed value once you have real data.
DEFAULT_THRESHOLD_C = 28.0
MIN_BLOB_AREA_PX = 4  # smaller than this is almost certainly noise, not a person


def frame_from_bytes(raw: bytes) -> np.ndarray:
    """
    Decode the UDP payload back into a (24, 32) float32 array.
    Matches the packing done on the ESP32 side — 768 little-endian
    float32 values, row-major.
    """
    arr = np.frombuffer(raw, dtype="<f4")
    if arr.size != MLX90640_COLS * MLX90640_ROWS:
        raise ValueError(f"expected {MLX90640_COLS*MLX90640_ROWS} floats, got {arr.size}")
    return arr.reshape((MLX90640_ROWS, MLX90640_COLS))


def find_warm_blob(frame: np.ndarray, threshold_c: float = DEFAULT_THRESHOLD_C):
    """
    Returns (cx, cy, area_px) for the largest warm blob, or None if
    nothing above threshold is found.
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
    """Top-level entry point: raw frame in, boresight error out (or None if no target)."""
    blob = find_warm_blob(frame, threshold_c)
    if blob is None:
        return None
    cx, cy, area = blob
    return compute_boresight_error(cx, cy, area)
