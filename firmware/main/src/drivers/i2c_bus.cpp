#include "i2c_bus.h"
#include <Wire.h>
#include <Arduino.h>

// ─── Maximum read retries before declaring a bus failure ─────────────────────
// One retry catches single-frame glitches (EMI from motor switching)
// without adding meaningful latency to the 500Hz control loop.
#define I2C_MAX_RETRIES 1

namespace i2c_bus {

// ─── init ─────────────────────────────────────────────────────────────────────
void init(int sdaPin, int sclPin, uint32_t frequencyHz, uint32_t timeoutMs) {
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(frequencyHz);

    // If the bus stalls (wire pulled, device reset mid-transaction),
    // this timeout releases it instead of freezing the flight loop forever.
    Wire.setTimeOut(timeoutMs);

    Serial.printf("[I2C] Initialized — SDA:%d SCL:%d  %luHz  timeout:%lums\n",
                  sdaPin, sclPin, frequencyHz, timeoutMs);
}

// ─── readBytes ────────────────────────────────────────────────────────────────
bool readBytes(uint8_t deviceAddr, uint8_t regAddr,
               uint8_t* outData,   uint8_t len) {

    for (int attempt = 0; attempt <= I2C_MAX_RETRIES; attempt++) {

        // Point the device's internal register pointer to regAddr
        Wire.beginTransmission(deviceAddr);
        Wire.write(regAddr);

        // endTransmission(false) = repeated start — keeps bus ownership
        // so no other master (or glitch) can grab the bus between write and read
        if (Wire.endTransmission(false) != 0) {
            // NACK or bus error — wait a little then retry
            delayMicroseconds(200);
            continue;
        }

        // Request exactly `len` bytes — true = send STOP after read
        uint8_t received = Wire.requestFrom(deviceAddr, len, (uint8_t)true);
        if (received != len) {
            delayMicroseconds(200);
            continue;
        }

        // Copy bytes out of the hardware FIFO
        for (uint8_t i = 0; i < len; i++) {
            outData[i] = Wire.read();
        }
        return true; // Success
    }

    // All attempts failed — caller must handle (imu.cpp sets isHealthy = false)
    return false;
}

// ─── writeByte ────────────────────────────────────────────────────────────────
bool writeByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t value) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);
    Wire.write(value);

    // endTransmission() returns 0 on success, non-zero on NACK/error
    bool ok = (Wire.endTransmission() == 0);
    if (!ok) {
        Serial.printf("[I2C] Write FAILED — addr:0x%02X reg:0x%02X val:0x%02X\n",
                      deviceAddr, regAddr, value);
    }
    return ok;
}

// ─── scan ─────────────────────────────────────────────────────────────────────
// Useful during hardware bring-up to confirm the MPU-6050 is alive at 0x68.
void scan() {
    Serial.println("[I2C] Scanning bus...");
    int found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C] Found device at 0x%02X\n", addr);
            found++;
        }
    }

    if (found == 0) {
        Serial.println("[I2C] No devices found — check wiring and pull-up resistors.");
    } else {
        Serial.printf("[I2C] Scan complete — %d device(s) found.\n", found);
    }
}

} // namespace i2c_bus