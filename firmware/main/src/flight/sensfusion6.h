#ifndef SENSFUSION6_H
#define SENSFUSION6_H

#include "flight_types.h"

// ─── SensFusion6 ──────────────────────────────────────────────────────────────
// Complementary filter for 6-axis IMU (gyro + accelerometer, no magnetometer).
// This is the ONLY place sensor fusion happens in the firmware.
// imu.cpp delivers raw calibrated values → sensfusion6 produces fused angles.
//
// Filter behaviour:
//   - Gyro:  fast, accurate short-term, drifts over time
//   - Accel: slow, noisy, but always knows gravity direction
//   - Blend: ALPHA trusts gyro, (1-ALPHA) corrects with accel each step
//
// Limitations:
//   - Yaw drifts with no magnetometer (acceptable for a 15s hover mission)
//   - Accel correction is skipped during high-G frames (vibration immunity)

class SensFusion6 {
public:
    // init: seed the filter with a known starting orientation.
    // Call after IMU calibration. Runs short accel-convergence pass internally.
    void init(float rollDeg, float pitchDeg, float yawDeg);

    // update: fuse one frame of calibrated gyro + accel data.
    // gxDps/gyDps/gzDps : calibrated gyro rates in degrees/second
    // axG/ayG/azG        : calibrated accel in G (gravity = 1.0 on Z when flat)
    // dt                 : time since last call in seconds
    void update(float gxDps, float gyDps, float gzDps,
                float axG,   float ayG,   float azG,
                float dt);

    // attitude: returns latest fused roll/pitch/yaw in degrees
    AttitudeDeg attitude() const;

    // resetYaw: zero the yaw reference (call after arming)
    void resetYaw();

private:
    AttitudeDeg _att;

    // Complementary filter coefficient (0–1).
    // 0.98 = trust gyro 98%, correct with accel 2% per step.
    // Increase for smoother but slower accel correction.
    // Decrease for faster levelling but more vibration sensitivity.
    static constexpr float ALPHA = 0.98f;

    // Accel magnitude guard: if total acceleration deviates from 1G
    // by more than this threshold, skip accel correction that frame.
    // Prevents vibration spikes from corrupting the angle estimate.
    static constexpr float ACCEL_MIN_G = 0.7f;  // below this → freefall / spike
    static constexpr float ACCEL_MAX_G = 1.3f;  // above this → hard impact / vibration
};

#endif // SENSFUSION6_H