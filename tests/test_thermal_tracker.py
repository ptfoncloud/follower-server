"""
Tests for frame decoding and blob detection. Uses synthetic frames built
in the test instead of real captures, so no hardware needed.
"""
from pathlib import Path

import numpy as np
import pytest
from app.thermal_tracker import (
    FRAME_BYTES, frame_from_bytes, frame_to_bytes, find_warm_blob, track,
    MLX90640_COLS, MLX90640_ROWS,
)

# written by follower-main's C++ encoder:
#   test/native/test_thermal_wire.cpp --golden golden_frame_776.bin
GOLDEN_PATH = Path(__file__).parent / "data" / "golden_frame_776.bin"


def golden_source_frame():
    """Same frame the C++ test builds, in float32."""
    rows, cols = np.indices((MLX90640_ROWS, MLX90640_COLS))
    f = (np.float32(20.0) + np.float32(0.4) * cols.astype(np.float32)
         + np.float32(0.1) * rows.astype(np.float32)).astype(np.float32)
    f[10:14, 14:18] = np.float32(33.5)
    f[0, 0] = np.nan
    return f


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
        raw = frame_to_bytes(original)
        assert len(raw) == FRAME_BYTES == 776
        decoded = frame_from_bytes(raw)
        assert decoded.shape == (MLX90640_ROWS, MLX90640_COLS)
        # two level frame, both levels come back exact
        np.testing.assert_array_almost_equal(original, decoded, decimal=4)

    def test_wrong_size_raises(self):
        with pytest.raises(ValueError):
            frame_from_bytes(b"\x00" * 100)  # too short

    @pytest.mark.parametrize("size", [0, 775, 777, 1460, 3072])
    def test_any_other_size_rejected(self, size):
        # 1460 = piece of an old raw frame, 3072 = the old raw float32 format
        with pytest.raises(ValueError):
            frame_from_bytes(b"\x00" * size)

    @pytest.mark.parametrize("min_c, scale_c", [
        (np.nan, 0.05), (20.0, np.nan), (np.inf, 0.05), (20.0, -0.05),
    ])
    def test_bad_header_rejected(self, min_c, scale_c):
        header = np.array([min_c, scale_c], dtype="<f4").tobytes()
        with pytest.raises(ValueError):
            frame_from_bytes(header + bytes(768))

    def test_quantization_error_within_half_step(self):
        rng = np.random.default_rng(1234)
        original = rng.uniform(18.0, 37.0, size=(MLX90640_ROWS, MLX90640_COLS)).astype(np.float32)
        raw = frame_to_bytes(original)
        scale = np.frombuffer(raw, dtype="<f4", count=2)[1]
        decoded = frame_from_bytes(raw)
        worst = np.max(np.abs(decoded - original))
        assert worst <= scale / 2 + 1e-4
        assert worst < 0.05  # ~19 C span -> ~0.037 C half-step

    def test_uniform_frame_decodes_exactly(self):
        original = make_frame(ambient=22.25)
        decoded = frame_from_bytes(frame_to_bytes(original))
        np.testing.assert_array_equal(decoded, original)

    def test_non_finite_pixel_encodes_as_coldest(self):
        original = make_frame(hot_region=(10, 14, 14, 18))
        original[0, 0] = np.nan
        decoded = frame_from_bytes(frame_to_bytes(original))
        assert decoded[0, 0] == pytest.approx(original[np.isfinite(original)].min())

    def test_tracker_still_finds_target_after_round_trip(self):
        frame = make_frame(hot_region=(10, 14, 14, 18))
        assert track(frame_from_bytes(frame_to_bytes(frame))) is not None


class TestGoldenFrameFromCppEncoder:
    """Decodes bytes produced by the real ESP32 encoder (thermal_wire.h)."""

    def test_golden_file_is_one_frame(self):
        assert GOLDEN_PATH.stat().st_size == FRAME_BYTES

    def test_golden_decodes_within_tolerance(self):
        raw = GOLDEN_PATH.read_bytes()
        min_c, scale_c = np.frombuffer(raw, dtype="<f4", count=2)
        assert min_c == pytest.approx(20.1, abs=1e-4)
        decoded = frame_from_bytes(raw)
        src = golden_source_frame()
        finite = np.isfinite(src)
        assert np.max(np.abs(decoded[finite] - src[finite])) <= scale_c / 2 + 1e-4
        assert decoded[0, 0] == pytest.approx(min_c)          # NaN pixel -> coldest
        assert decoded[11, 15] > 33.0                          # hot block survives

    def test_python_encoder_matches_cpp_byte_for_byte(self):
        assert frame_to_bytes(golden_source_frame()) == GOLDEN_PATH.read_bytes()


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
        frame[5, 5] = 40.0  # one hot pixel, should get filtered as noise
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
