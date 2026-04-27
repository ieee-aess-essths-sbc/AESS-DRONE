#ifndef PID_ADVANCED_H
#define PID_ADVANCED_H

#include <Arduino.h>

// ─── PidAdvanced ──────────────────────────────────────────────────────────────
// Single-axis PID controller with:
//   - Anti-windup integral clamping
//   - D-term fed from external rate (gyro) to avoid differentiation noise
//   - Output limiting
//   - Live dt update support
class PidAdvanced {
public:
    // Configure gains and initial dt
    void init(float kp, float ki, float kd, float dt);

    // Update gains/limits at runtime if needed
    void setDt(float dt);
    void setDesired(float desired);
    void setIntegralLimit(float limit);
    void setOutputLimit(float limit);

    // Standard update: PID from measured angle alone (D = finite difference)
    float update(float measured);

    // Preferred update: D-term from external gyro rate (less noisy)
    // Pass the gyro rate that corresponds to this axis (gx for roll, gy for pitch)
    float updateWithRate(float measured, float measuredRate);

    // Wipe integral and previous error (call on arm/disarm)
    void reset();

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;
    float _dt = 0.004f;

    float _desired       = 0.0f;
    float _integral      = 0.0f;
    float _prevError     = 0.0f;
    float _integralLimit = 50.0f;  // Default anti-windup cap
    float _outputLimit   = 200.0f; // Default output cap (duty-cycle units)
};

#endif // PID_ADVANCED_H