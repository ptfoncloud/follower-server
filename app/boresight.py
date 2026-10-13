"""
boresight.py - pure math, no I/O. Turns a blob's pixel position into
an angle in degrees using the MLX90640's 55 deg FOV.

Kept out of the networking/OpenCV code so it's easy to test on its own.
"""
from dataclasses import dataclass

# Adafruit MLX90640 55 deg version (#4407), the part in the BOM
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
    Pixel offset from frame center -> angle, assuming a roughly linear
    mapping across the FOV. Fine for 55 deg at this resolution, wouldn't
    be right for a fisheye where it's non linear near the edges.
    """
    normalized = pixel_offset / (frame_size / 2.0)   # -1..1 across the frame
    return normalized * (fov_deg / 2.0)


def compute_boresight_error(
    blob_cx: float, blob_cy: float, blob_area_px: int,
    frame_cols: int = MLX90640_COLS, frame_rows: int = MLX90640_ROWS,
    fov_deg: float = MLX90640_FOV_DEG,
) -> BoresightError:
    """
    blob_cx/cy: blob centroid in pixels (0,0 top left, same as OpenCV).
    blob_area_px: blob size in pixels, only used for the rough confidence
    number, not the angle.
    """
    center_x = frame_cols / 2.0
    center_y = frame_rows / 2.0

    offset_x = blob_cx - center_x
    offset_y = blob_cy - center_y

    pan_deg = pixel_to_angle(offset_x, frame_cols, fov_deg)
    tilt_deg = pixel_to_angle(offset_y, frame_rows, fov_deg)

    # Rough confidence: a blob that's a sane fraction of the frame (a
    # person a few metres away) scores high, a 1-2 px noise blob or one
    # filling the whole frame (saturated / something right up close)
    # scores low. Both bounds are guesses until there's real data.
    frame_area = frame_cols * frame_rows
    area_fraction = blob_area_px / frame_area
    if area_fraction < 0.01 or area_fraction > 0.6:
        confidence = 0.2
    else:
        confidence = min(1.0, area_fraction * 8.0)

    return BoresightError(pan_deg=pan_deg, tilt_deg=tilt_deg, confidence=confidence)
