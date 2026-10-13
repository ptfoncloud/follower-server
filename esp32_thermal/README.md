# Thermal streaming

MLX90640 reading + wifi UDP streaming for the ESP32, feeds the
follower-server boresight service on Henri. Not hooked into main.cpp
yet.

## Hooking it into main.cpp

Wifi creds go in `src/secrets.h` (gitignored). Make it from the template
first:

```powershell
Copy-Item src\secrets.example.h src\secrets.h   # then fill it in
```

```cpp
#include "secrets.h"         // WIFI_SSID, WIFI_PASSWORD, THERMAL_SERVER_*
#include "thermal_stream.h"

ThermalStream thermal;

void setup() {
    // ... existing setup ...
    thermal.begin(WIFI_SSID, WIFI_PASSWORD,
                  THERMAL_SERVER_IP,     // Henri
                  THERMAL_SERVER_PORT);  // UDP_LISTEN_PORT in server.py
}

void loop() {
    // ... existing loop ...
    thermal.update();  // rate limits itself, fine to call every loop
}
```

### Build setup

Needs the Adafruit MLX90640 lib. PlatformIO's dependency finder doesn't
pick up the framework libs (WiFi, Wire, SPI) through headers outside
`src/`, so list them. This config compiles for esp32dev:

```ini
lib_deps =
    adafruit/Adafruit MLX90640 @ ^1.1.1
    WiFi
    Wire
    SPI
lib_ldf_mode = deep+
build_flags = -I esp32_thermal
```

## Frame format (one UDP packet per frame, 776 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | float32 LE | `min_temp_c`, lowest finite pixel |
| 4 | float32 LE | `scale_c`, °C per step = `(max - min) / 255` |
| 8 | uint8 x 768 | `q`, row major 24 x 32 |

`temp_c = min_temp_c + q * scale_c`. Worst case error is `scale_c / 2`
(about 0.03 °C over a 15 °C range). NaN/inf pixels get sent as `q = 0`
(coldest) so a bad read can't look like a warm target. Encoder is
`thermal_wire.h`, tested in `test/native/test_thermal_wire.cpp`. Decoder
is `frame_from_bytes` in follower-server's `app/thermal_tracker.py`.

Why not raw floats: 768 float32 = 3072 bytes, and arduino-esp32's
WiFiUDP sends a packet every time its 1460 byte buffer fills
(`write(uint8_t)` in `WiFiUdp.cpp`). So a raw frame went out as 3
packets (1460 + 1460 + 152) and the server threw all of them out for
being the wrong size. Same code in arduino-esp32 2.0.17 (what PlatformIO
espressif32 7.x installs) and current master (`NetworkUdp.cpp`).

## Wiring

MLX90640 on the ESP32's I2C, default pins GPIO21 SDA / GPIO22 SCL.
Separate from the pixy's SPI and the FC uart so nothing conflicts.

## Before flying with it

Bench test with everything running (pixy tracking, FC connected,
thermal streaming) and make sure the 10 Hz control loop doesn't slow
down. Wifi calls can block sometimes and nobody's measured whether that
ever delays a velocity command. If it does, move the thermal stuff to
the second core with `xTaskCreatePinnedToCore` so it can't stall the
flight loop. Probably worth doing before it flies either way.
