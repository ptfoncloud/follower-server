#pragma once
/*  thermal_wire.h - MLX90640 frame format, ESP32 -> follower-server.
    Plain C++ with no Arduino deps so it's tested on desktop
    (test/native/test_thermal_wire.cpp).

    Why not raw floats: 768 float32 = 3072 bytes but arduino-esp32's
    WiFiUDP sends a packet whenever its 1460 byte buffer fills
    (WiFiUdp.cpp write(uint8_t): `if (tx_buffer_len == 1460) endPacket();`)
    so a raw frame went out as 3 packets (1460 + 1460 + 152) and the
    server rejected all of them. This fits in one.

    Format (776 bytes, little endian, one UDP packet per frame):
      offset 0  float32  min_temp_c   lowest finite pixel temperature
      offset 4  float32  scale_c      degrees C per quantization step (>= 0)
      offset 8  uint8[768]  q         row-major, 24 rows x 32 cols
    Decode:  temp_c = min_temp_c + q * scale_c
    Encode:  scale_c = (max - min) / 255, q = round((t - min) / scale_c)
    Max error is scale_c / 2 (~0.03 C over a 15 C range).
    NaN/Inf pixels from a bad read get q = 0 (coldest) so they can't look
    like a warm target.
    Has to stay in sync with follower-server app/thermal_tracker.py. */

#include <stdint.h>
#include <string.h>
#include <math.h>

#define THERMAL_WIRE_PIXELS        768   // 32 x 24
#define THERMAL_WIRE_HEADER_BYTES  8
#define THERMAL_WIRE_FRAME_BYTES   (THERMAL_WIRE_HEADER_BYTES + THERMAL_WIRE_PIXELS)  // 776

// Encodes one frame into out[THERMAL_WIRE_FRAME_BYTES]. Returns false (and
// leaves out untouched) if no pixel is finite.
inline bool thermal_wire_encode(const float* frame, uint8_t* out) {
    float lo = INFINITY, hi = -INFINITY;
    for (int i = 0; i < THERMAL_WIRE_PIXELS; i++) {
        float t = frame[i];
        if (!isfinite(t)) continue;
        if (t < lo) lo = t;
        if (t > hi) hi = t;
    }
    if (!isfinite(lo)) return false;

    float scale = (hi - lo) / 255.0f;
    memcpy(out,     &lo,    4);   // ESP32 and x86 are both little-endian
    memcpy(out + 4, &scale, 4);

    uint8_t* q = out + THERMAL_WIRE_HEADER_BYTES;
    for (int i = 0; i < THERMAL_WIRE_PIXELS; i++) {
        float t = frame[i];
        if (!isfinite(t) || scale <= 0.0f) { q[i] = 0; continue; }
        long v = lroundf((t - lo) / scale);
        q[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
    }
    return true;
}
