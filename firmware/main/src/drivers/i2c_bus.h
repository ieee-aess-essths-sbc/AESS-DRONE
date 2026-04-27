#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>

// ─── i2c_bus ──────────────────────────────────────────────────────────────────
// Centralized I2C access layer for all peripherals.
// ALL I2C communication in this firmware goes through here — never call
// Wire.beginTransmission() directly from imu.cpp or any other module.
// This gives us one place to handle errors, retries, and timeouts.

namespace i2c_bus {

// ─── init ─────────────────────────────────────────────────────────────────────
// Call once in main setup() before any peripheral init.
// sdaPin/sclPin   : GPIO numbers (ESP32-S3 Zero: SDA=3, SCL=4)
// frequencyHz     : 400000 for Fast Mode (recommended for MPU-6050)
// timeoutMs       : how long to wait before declaring the bus frozen
//                   50ms is safe — longer than any valid transaction,
//                   short enough not to stall the flight loop meaningfully
void init(int sdaPin, int sclPin, uint32_t frequencyHz, uint32_t timeoutMs = 50);

// ─── readBytes ────────────────────────────────────────────────────────────────
// Burst-read `len` bytes from `regAddr` on `deviceAddr`.
// Uses repeated-start (endTransmission(false)) to keep bus ownership.
// Retries once on transient failure before returning false.
// Returns true if all `len` bytes were received successfully.
bool readBytes(uint8_t deviceAddr, uint8_t regAddr,
               uint8_t* outData,   uint8_t len);

// ─── writeByte ────────────────────────────────────────────────────────────────
// Write a single byte to a register — used for MPU-6050 configuration.
// Returns true if the device ACKed the transmission.
bool writeByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t value);

// ─── scan ─────────────────────────────────────────────────────────────────────
// Scans all 127 I2C addresses and prints found devices to Serial.
// Call from setup() during development to verify MPU-6050 is at 0x68.
// Remove or ifdef-out in production builds.
void scan();

} // namespace i2c_bus

#endif // I2C_BUS_H