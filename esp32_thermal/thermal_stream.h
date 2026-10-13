#pragma once
/*  thermal_stream - MLX90640 read + wifi UDP frame streaming.

    Read before wiring this in: main.cpp already has SPI to the pixy,
    UART to the FC and a 10 Hz control loop. Adding I2C + wifi to the
    same loop is real extra load. The MLX90640 is slow (2-8 Hz) so it
    won't fight the loop much, but wifi send calls can block for tens of
    ms sometimes and nobody has measured whether that delays a velocity
    command. Time the control loop on the bench before flying with it.

    If it does cause timing problems run this on the second core
    (xTaskCreatePinnedToCore) so it can't stall the flight loop. Not done
    yet to keep the first version simple. */

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
        // 4 Hz, fast enough to track with, slow enough not to hog the I2C bus
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

    // call from loop(), it rate limits itself to the sensor refresh rate
    void update() {
        if (!ready) return;
        uint32_t now = millis();
        if (now - lastReadMs < 250) return;  // matches 4Hz refresh above
        lastReadMs = now;

        if (mlx.getFrame(frameBuf) != 0) {
            Serial.println("[THERMAL] Frame read failed, skipping");
            return;
        }

        // 776 byte frame (thermal_wire.h). has to stay under WiFiUDP's 1460
        // byte buffer or it gets split into multiple packets
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
