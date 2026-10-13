# Thermal Streaming Module

Adds MLX90640 reading + WiFi UDP streaming to the ESP32 companion
controller, feeding the `follower_server` OpenCV/boresight service
running on Henri.

## Integration into main.cpp

```cpp
#include "thermal_stream.h"

ThermalStream thermal;

void setup() {
    // ... existing setup ...
    thermal.begin("YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD",
                   "192.168.1.169",  // Henri's IP
                   14600);           // matches UDP_LISTEN_PORT in server.py
}

void loop() {
    // ... existing tracking/MAVLink loop ...
    thermal.update();  // internally rate-limited, safe to call every iteration
}
```

### Build setup

`thermal_stream.h` needs the Adafruit MLX90640 library, and PlatformIO's
default dependency finder doesn't discover framework libraries (WiFi,
Wire, SPI) through headers outside `src/`. A configuration that was
compile-tested for `esp32dev`:

```ini
lib_deps =
    adafruit/Adafruit MLX90640 @ ^1.1.1
    WiFi
    Wire
    SPI
lib_ldf_mode = deep+
build_flags = -I esp32_thermal
```

## Wire format (one UDP datagram per frame, 776 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | float32 LE | `min_temp_c` — lowest finite pixel |
| 4 | float32 LE | `scale_c` — °C per step, `(max - min) / 255` |
| 8 | uint8 × 768 | `q` — row-major 24 × 32 |

`temp_c = min_temp_c + q * scale_c`. Worst-case error is `scale_c / 2`
(about 0.03 °C across a 15 °C span). Non-finite pixels are sent as
`q = 0` (the coldest value) so a bad read can't look like a warm
target. Encoder: `thermal_wire.h`, tested by
`test/native/test_thermal_wire.cpp`. Decoder: follower-server's
`app/thermal_tracker.py::frame_from_bytes`.

**Why not raw floats:** 768 float32 is 3072 bytes, and arduino-esp32's
`WiFiUDP` sends its buffer as a datagram every time it fills to 1460
bytes (`WiFiUdp.cpp`, `write(uint8_t)`). A raw frame left as three
datagrams (1460 + 1460 + 152 bytes), and the server rejected every one
as the wrong size. Verified in arduino-esp32 2.0.17 (what PlatformIO's
`espressif32` 7.x installs) and in current master's `NetworkUdp.cpp`.

## Wiring

MLX90640 shares the I2C bus (SDA/SCL) — separate from Pixy2's SPI bus
and the FC's UART, so no pin conflicts. Standard ESP32 I2C default pins
(GPIO 21 SDA, GPIO 22 SCL) unless you've remapped them elsewhere.

## Before trusting this in flight

Bench test with the full stack running — Pixy2 tracking active, FC
connected, thermal streaming active — and confirm the control loop's
10Hz timing doesn't degrade. WiFi calls occasionally block; whether
that ever delays a velocity command to the FC hasn't been measured. If
it does, the real fix is running thermal streaming on the ESP32's
second core via `xTaskCreatePinnedToCore` so it physically cannot
stall the flight-critical loop — worth doing proactively rather than
finding out the hard way mid-flight.
