"""
Tests for frame decoding and blob detection. Uses synthetic frames
(numpy arrays built in-test) rather than real sensor captures — no
hardware dependency needed to run these.
"""
import numpy as np
import pytest
from app.thermal_tracker import frame_from_bytes, find_warm_blob, track, MLX90640_COLS, MLX90640_ROWS


def make_frame(hot_region=None, ambient=20.0, hot_temp=32.0):
    """Build a synthetic 32x24 frame, optionally with a hot rectangular
    region at (row_start, row_end, col_start, col_end)."""
    frame = np.full((MLX90640_ROWS, MLX90640_COLS), ambient, dtype=np.float32)
    if hot_region:
        r0, r1, c0, c1 = hot_region
        frame[r0:r1, c0:c1] = hot_temp
    return frame


class TestFrameFromBytes:
    def test_round_trip(self):
        original = make_frame(hot_region=(10, 14, 12, 20))
        raw = original.astype("<f4").tobytes()
        decoded = frame_from_bytes(raw)
        np.testing.assert_array_almost_equal(original, decoded)

    def test_wrong_size_raises(self):
        with pytest.raises(ValueError):
            frame_from_bytes(b"\x00" * 100)  # too short


class TestFindWarmBlob:
    def test_no_blob_when_nothing_warm(self):
        frame = make_frame()  # uniform ambient, nothing above threshold
        assert find_warm_blob(frame) is None

    def test_finds_centred_hot_region(self):
        frame = make_frame(hot_region=(10, 14, 14, 18))
        result = find_warm_blob(frame)
        assert result is not None
        cx, cy, area = result
        # centroid should land roughly in the middle of the hot region
        assert 14 <= cx <= 18
        assert 10 <= cy <= 14
        assert area > 0

    def test_ignores_single_pixel_noise(self):
        frame = make_frame()
        frame[5, 5] = 40.0  # one hot pixel — should be filtered as noise
        assert find_warm_blob(frame) is None


class TestTrackEndToEnd:
    def test_no_target_returns_none(self):
        frame = make_frame()
        assert track(frame) is None

    def test_target_present_returns_boresight_error(self):
        frame = make_frame(hot_region=(10, 14, 14, 18))  # roughly centred
        result = track(frame)
        assert result is not None
        assert abs(result.pan_deg) < 10   # roughly centred target = small error
        assert abs(result.tilt_deg) < 10
