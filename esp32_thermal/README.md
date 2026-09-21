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
