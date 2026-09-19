"""
Unit tests for the boresight math — pure functions, no I/O, no mocking
needed. This is the piece worth having watertight before anything else
in this service, since every other test here depends on it being right.
"""
import pytest
from app.boresight import (
    compute_boresight_error, pixel_to_angle,
    MLX90640_FOV_DEG, MLX90640_COLS, MLX90640_ROWS,
)


class TestPixelToAngle:
    def test_dead_centre_is_zero_error(self):
        assert pixel_to_angle(0, MLX90640_COLS, MLX90640_FOV_DEG) == 0.0

    def test_full_right_edge_is_half_fov(self):
        # offset of frame_size/2 = the very edge of the frame = half the FOV
        result = pixel_to_angle(MLX90640_COLS / 2, MLX90640_COLS, MLX90640_FOV_DEG)
        assert result == pytest.approx(MLX90640_FOV_DEG / 2, abs=0.01)

    def test_negative_offset_gives_negative_angle(self):
        result = pixel_to_angle(-MLX90640_COLS / 2, MLX90640_COLS, MLX90640_FOV_DEG)
        assert result == pytest.approx(-MLX90640_FOV_DEG / 2, abs=0.01)


class TestComputeBoresightError:
    def test_target_dead_centre_gives_zero_pan_and_tilt(self):
        cx, cy = MLX90640_COLS / 2, MLX90640_ROWS / 2
        result = compute_boresight_error(cx, cy, blob_area_px=40)
        assert result.pan_deg == pytest.approx(0.0, abs=0.1)
        assert result.tilt_deg == pytest.approx(0.0, abs=0.1)

    def test_target_right_of_centre_gives_positive_pan(self):
        cx, cy = MLX90640_COLS * 0.9, MLX90640_ROWS / 2
        result = compute_boresight_error(cx, cy, blob_area_px=40)
        assert result.pan_deg > 0

    def test_target_left_of_centre_gives_negative_pan(self):
        cx, cy = MLX90640_COLS * 0.1, MLX90640_ROWS / 2
        result = compute_boresight_error(cx, cy, blob_area_px=40)
        assert result.pan_deg < 0

    def test_target_below_centre_gives_positive_tilt(self):
        cx, cy = MLX90640_COLS / 2, MLX90640_ROWS * 0.9
        result = compute_boresight_error(cx, cy, blob_area_px=40)
        assert result.tilt_deg > 0

    def test_tiny_blob_gets_low_confidence(self):
        result = compute_boresight_error(16, 12, blob_area_px=1)
        assert result.confidence < 0.5

    def test_saturated_frame_gets_low_confidence(self):
        # blob covering most of the frame - likely sensor saturation or
        # something wrong, not a real tracked target at a sane distance
        huge_area = int(MLX90640_COLS * MLX90640_ROWS * 0.9)
        result = compute_boresight_error(16, 12, blob_area_px=huge_area)
        assert result.confidence < 0.5

    def test_reasonable_blob_gets_decent_confidence(self):
        # a blob taking up a sane fraction of frame - a person at a few metres
        reasonable_area = int(MLX90640_COLS * MLX90640_ROWS * 0.08)
        result = compute_boresight_error(16, 12, blob_area_px=reasonable_area)
        assert result.confidence > 0.5
