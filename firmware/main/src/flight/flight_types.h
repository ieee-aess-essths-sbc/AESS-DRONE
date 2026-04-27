#ifndef FLIGHT_TYPES_H
#define FLIGHT_TYPES_H

// ─── flight_types.h ───────────────────────────────────────────────────────────
// Shared data structures used across imu, controller, power_distrib, stabilizer.
// No logic here — pure data definitions only.

// ─── Attitude (Euler angles in degrees) ──────────────────────────────────────
struct AttitudeDeg {
    float roll  = 0.0f;  // + = right side down
    float pitch = 0.0f;  // + = nose down
    float yaw   = 0.0f;  // + = rotate CCW (gyro-integrated, drifts)
};

// ─── Angular Rate (degrees per second, from calibrated gyro) ─────────────────
struct AttitudeRateDps {
    float roll  = 0.0f;  // gx from IMU
    float pitch = 0.0f;  // gy from IMU
    float yaw   = 0.0f;  // gz from IMU
};

// ─── Setpoint ─────────────────────────────────────────────────────────────────
// What the drone is trying to achieve this control cycle.
// For autonomous hover: roll=0, pitch=0, yaw rate=0.
struct Setpoint {
    AttitudeDeg     attitudeDeg;     // Target Euler angles
    AttitudeRateDps attitudeRateDps; // Target angular rates (yaw uses rate control)
};

// ─── ControlOutput ────────────────────────────────────────────────────────────
// Output of the PID controller — signed correction values in duty-cycle units.
// These are DELTAS, not absolute motor speeds.
// Passed to PowerDistribution::mixX() alongside a separate base thrust value.
struct ControlOutput {
    float roll  = 0.0f;  // + = increase left motors, decrease right
    float pitch = 0.0f;  // + = increase front motors, decrease rear
    float yaw   = 0.0f;  // + = increase CCW motors, decrease CW
    // NOTE: No thrust here — thrust is owned by main and passed to mixX() directly
};

#endif // FLIGHT_TYPES_H