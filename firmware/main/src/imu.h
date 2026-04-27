#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// ─── IMUData ──────────────────────────────────────────────────────────────────
// Raw calibrated sensor output — NO fused angles here.
// Fusion is handled exclusively by SensFusion6 in the stabilizer pipeline.
struct IMUData {
    float ax, ay, az;  // Accelerometer in G  (gravity = ~1.0 on Z when flat)
    float gx, gy, gz;  // Gyroscope in deg/s  (zeroed by calibration)
    bool  isHealthy;   // false → stabilizer must trigger emergency stop
};

// ─── Public API ──────────────────────────────────────────────────────────────
void initIMU();      // Init I2C via i2c_bus, configure MPU-6050, run calibration
void calibrateIMU(); // Collect bias offsets — drone must be flat and still
IMUData readIMU();   // Read 14 bytes, apply offsets, return calibrated values

#endif // IMU_H