# follower-server

![Python](https://img.shields.io/badge/python-3.11-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.9-green)
![Tests](https://img.shields.io/badge/tests-17%2F17-brightgreen)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

Runs on Henri. Receives raw MLX90640 thermal frames from the drone
over a UDP uplink, runs OpenCV blob detection to find the tracked
subject, and computes real boresight error — pan/tilt in actual
degrees, using the sensor's confirmed 55° field of view, not just a
raw pixel offset.

![Architecture](docs/assets/architecture.svg)

Part of a three-repo system: the flight firmware and safety logic live
in [follower-main](https://github.com/ptfoncloud/follower-main); the
ground station dashboard lives in
[gcs-ground-control](https://github.com/ptfoncloud/gcs-ground-control).
Full system-level design rationale — including the choices that touch
this service directly (why UDP, why real angular error instead of raw
pixels, where sensor fusion deliberately stops for now) — lives in
[follower-main's ENGINEERING_DECISIONS.md](https://github.com/ptfoncloud/follower-main/blob/main/docs/ENGINEERING_DECISIONS.md)
rather than being duplicated here.

## What this is, and what it deliberately is not

This is a **secondary, monitoring/cross-check tracking channel**. The
drone's primary flight control comes from `follower-main`'s onboard
Pixy2 tracking, safety-gated against the FC's real armed/mode state.
This service does not command the vehicle — it publishes target data
over WebSocket for the dashboard to display, and as a foundation for
deliberate sensor fusion later, not an accidental one now.

## Running it

```bash
pip install -r requirements.txt
python -m app.server
```

Point the ESP32's thermal streaming module (`esp32_thermal/` — the
canonical, maintained copy lives in `follower-main`, mirrored here for
reference) at Henri's IP on port 14600.

## Tests

```bash
python -m pytest tests/ -v
```

17 tests, currently passing. `boresight.py`'s angle math is fully unit
tested as pure functions (no I/O); `thermal_tracker.py`'s blob
detection is tested against synthetic frames, so nothing here needs
real sensor hardware to verify the logic is correct.

## Known limitations

- **Fixed threshold** (`DEFAULT_THRESHOLD_C = 28.0`) — a bench-tested
  starting point, will need real-world tuning. An adaptive threshold
  (mean + N·stddev per frame) is the obvious next step, not built yet.
- **Single-blob tracking** — locks onto the largest warm object in
  frame; no multi-target handling.
- **No temporal filtering** — each frame processed independently. A
  simple moving average or Kalman filter on the output would smooth
  sensor noise; not implemented.

## License

MIT — see [LICENSE](LICENSE).
