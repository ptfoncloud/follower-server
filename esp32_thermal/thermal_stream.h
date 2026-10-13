#pragma once
/*  ================================================================
    thermal_stream — MLX90640 read + WiFi UDP frame streaming.

    HONEST NOTE ON SCOPE if you use this code, read before wiring this in:
    The main follower firmware (main.cpp) already has its hands full —
    SPI to Pixy2, UART to the FC, and a 10Hz control loop with hard
    timing requirements. Adding I2C (MLX90640) AND WiFi (UDP send) to
    that same loop is a real additional load, not a trivial add. The
    MLX90640 itself is slow (2-8Hz typical refresh depending on config)
    so it won't fight the 10Hz control loop for CPU time constantly,
    but WiFi's connection/send calls CAN block for tens of milliseconds
    occasionally — untested here whether that ever causes a late
    velocity command to the FC. Bench-test with a stopwatch on the
    control loop's actual timing before trusting this in the air.

    If it DOES cause timing problems, the fix is running this on ESP32's
    second core (Arduino framework exposes FreeRTOS tasks — xTaskCreatePinnedToCore)
    so thermal streaming can't stall the flight-critical loop. Not done
    here to keep the first version simple; flag if you want that added.
    ================================================================ */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MLX90640.h>
#include "thermal_wire.h"

#define MLX_COLS 32
#define MLX_ROWS 24
#define MLX_FRAME_FLOATS (MLX_COLS * MLX_ROWS)

class ThermalStream {
public:
    bool begin(const char* wifiSsid, const char* wifiPass,
               const char* serverIp, uint16_t serverPort) {
        if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
            Serial.println("[THERMAL] MLX90640 not found on I2C!");
            return false;
        }
        // 4Hz refresh — a reasonable middle ground: fast enough to be
        // useful for tracking, slow enough not to dominate I2C bus time
        mlx.setRefreshRate(MLX90640_4_HZ);
        mlx.setResolution(MLX90640_ADC_18BIT);

        Serial.printf("[THERMAL] Connecting to WiFi '%s'...\n", wifiSsid);
        WiFi.begin(wifiSsid, wifiPass);
        uint32_t startMs = millis();
        while (WiFi.status() != WL_CONNECTED) {
            delay(250);
            if (millis() - startMs > 15000) {
                Serial.println("[THERMAL] WiFi connect timeout — thermal streaming disabled");
                return false;
            }
        }
        Serial.printf("[THERMAL] WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

        destIp.fromString(serverIp);
        destPort = serverPort;
        udp.begin(0);  // any local port for sending
        ready = true;
        return true;
    }

    /*  Call periodically from loop() — internally rate-limits to the
        sensor's own refresh rate, safe to call every iteration. */
    void update() {
        if (!ready) return;
        uint32_t now = millis();
        if (now - lastReadMs < 250) return;  // matches 4Hz refresh above
        lastReadMs = now;

        if (mlx.getFrame(frameBuf) != 0) {
            Serial.println("[THERMAL] Frame read failed, skipping");
            return;
        }

        // 776-byte quantized frame (see thermal_wire.h). Must stay under
        // WiFiUDP's 1460-byte tx buffer or it gets split across datagrams.
        static_assert(THERMAL_WIRE_FRAME_BYTES <= 1460, "frame must fit one WiFiUDP datagram");
        if (!thermal_wire_encode(frameBuf, wireBuf)) {
            Serial.println("[THERMAL] Frame has no finite pixels, skipping");
            return;
        }
        udp.beginPacket(destIp, destPort);
        udp.write(wireBuf, sizeof(wireBuf));
        udp.endPacket();
    }

    bool isReady() const { return ready; }

private:
    Adafruit_MLX90640 mlx;
    WiFiUDP udp;
    IPAddress destIp;
    uint16_t destPort = 0;
    bool ready = false;
    uint32_t lastReadMs = 0;
    float frameBuf[MLX_FRAME_FLOATS];
    uint8_t wireBuf[THERMAL_WIRE_FRAME_BYTES];
};
