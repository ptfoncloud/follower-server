# follower-server

![Python](https://img.shields.io/badge/python-3.11-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.9-green)
![Tests](https://img.shields.io/badge/tests-33%2F33-brightgreen)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

Runs on Henri. Gets MLX90640 thermal frames from the drone over UDP,
finds the warm blob with OpenCV, and works out the boresight error as
actual pan/tilt degrees using the sensor's 55° FOV instead of a raw
pixel offset.

![Architecture](docs/assets/architecture.svg)

Other repos: flight firmware and the safety gate are in
[follower-main](https://github.com/ptfoncloud/follower-main), ground
station is in
[gcs-ground-control](https://github.com/ptfoncloud/gcs-ground-control).
The design reasoning that affects this service (why UDP, why degrees,
why no sensor fusion yet) is in
[follower-main's ENGINEERING_DECISIONS.md](https://github.com/ptfoncloud/follower-main/blob/main/docs/ENGINEERING_DECISIONS.md).

## What it is / isn't

It's a secondary tracking channel for monitoring and cross checking.
The drone actually flies off the pixy2 tracker in follower-main, which
is gated on the FC's armed/mode state. This service never commands the
vehicle, it just publishes target data over a websocket for the
dashboard (and maybe sensor fusion later, done properly).

## Running

```bash
pip install -r requirements.txt
python -m app.server
```

Point the ESP32's thermal streaming (`esp32_thermal/`, the real copy is
in follower-main, this one is just for reference) at Henri's IP on port
14600.

## Frame format (UDP :14600)

One packet per frame, exactly 776 bytes, little endian:

| Offset | Type | Field |
|---|---|---|
| 0 | float32 | `min_temp_c`, lowest finite pixel |
| 4 | float32 | `scale_c`, °C per step = `(max - min) / 255` |
| 8 | uint8 x 768 | `q`, row major 24 rows x 32 cols |

`temp_c = min_temp_c + q * scale_c`. Worst case error is `scale_c / 2`
(about 0.03 °C over 15 °C). Anything that isn't exactly 776 bytes with a
finite non negative header gets rejected and logged.

The original format was raw float32 (3072 bytes) and it never actually
worked. arduino-esp32's WiFiUDP sends a packet every 1460 bytes, so each
frame showed up as 3 packets and all of them got rejected. The decoder
is `frame_from_bytes` in `app/thermal_tracker.py`. `frame_to_bytes` is a
python version of the ESP32 encoder (`esp32_thermal/thermal_wire.h` in
follower-main) and `tests/data/golden_frame_776.bin` is real output from
the C++ encoder that the tests decode and match byte for byte.

## Tests

```bash
python -m pytest tests/ -v
```

33 tests. The boresight math is tested as plain functions, blob
detection against synthetic frames, and the 776 byte decoder for round
trip, bad sizes/headers, quantization error and the golden frame from
the C++ encoder. No hardware needed. The UDP listener and websocket
broadcaster in `server.py` don't have automated tests yet.

## Known limitations

- Fixed threshold (`DEFAULT_THRESHOLD_C = 28.0`). It's a bench starting
  point and will need tuning. An adaptive threshold (mean + N*stddev
  per frame) is the obvious next step.
- Single blob, it locks onto the biggest warm thing in frame.
- No filtering over time, every frame is processed on its own. A moving
  average or kalman filter on the output would smooth out the noise.

## License

MIT, see [LICENSE](LICENSE).
