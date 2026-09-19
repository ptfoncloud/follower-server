"""
boresight.py — pure math, no I/O. Converts a thermal blob's pixel
position into a real angular error (degrees), using the MLX90640's
actual confirmed field of view.

Kept separate from the networking/OpenCV code specifically so it's
trivially unit-testable — this is the piece an interviewer is most
likely to ask you to walk through, and the piece most worth having
clean, isolated tests for.
"""
from dataclasses import dataclass

# Real, confirmed spec — Adafruit MLX90640 55-degree version (#4407),
# the exact part in the BOM. NOT a guess.
MLX90640_FOV_DEG = 55.0
MLX90640_COLS = 32
MLX90640_ROWS = 24


@dataclass
class BoresightError:
    pan_deg: float   # positive = target is to the RIGHT of centre
    tilt_deg: float  # positive = target is BELOW centre
    confidence: float  # 0-1, based on blob size relative to frame


def pixel_to_angle(pixel_offset: float, frame_size: int, fov_deg: float) -> float:
    """
    Convert a pixel offset from frame centre into a real angular offset,
    assuming a roughly linear (small-angle) mapping across the sensor's
    field of view. Good enough for a 55-degree FOV at this resolution —
    NOT accurate for a fisheye/wide-angle lens, where the mapping is
    genuinely non-linear near the edges.
    """
    normalized = pixel_offset / (frame_size / 2.0)   # -1..1 across the frame
    return normalized * (fov_deg / 2.0)


def compute_boresight_error(
    blob_cx: float, blob_cy: float, blob_area_px: int,
    frame_cols: int = MLX90640_COLS, frame_rows: int = MLX90640_ROWS,
    fov_deg: float = MLX90640_FOV_DEG,
) -> BoresightError:
    """
    blob_cx/cy: centroid of the detected warm blob, in pixel coordinates
    (0,0 at top-left, matching OpenCV convention).
    blob_area_px: blob size in pixels — used only to produce a rough
    confidence figure, not for the angle math itself.
    """
    center_x = frame_cols / 2.0
    center_y = frame_rows / 2.0

    offset_x = blob_cx - center_x
    offset_y = blob_cy - center_y

    pan_deg = pixel_to_angle(offset_x, frame_cols, fov_deg)
    tilt_deg = pixel_to_angle(offset_y, frame_rows, fov_deg)

    # Crude confidence heuristic: a blob taking up a sane fraction of the
    # frame (a person at a few metres) scores high; a 1-2px noise blob or
    # a blob filling the whole frame (sensor saturation / very close
    # object) scores low. Tune the two bounds once you have real data —
    # these are starting points, not calibrated thresholds.
    frame_area = frame_cols * frame_rows
    area_fraction = blob_area_px / frame_area
    if area_fraction < 0.01 or area_fraction > 0.6:
        confidence = 0.2
    else:
        confidence = min(1.0, area_fraction * 8.0)

    return BoresightError(pan_deg=pan_deg, tilt_deg=tilt_deg, confidence=confidence)
