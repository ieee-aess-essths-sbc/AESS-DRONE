#include "pid_advanced.h"

// ─── init ─────────────────────────────────────────────────────────────────────
void PidAdvanced::init(float kp, float ki, float kd, float dt) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _dt = dt;
    reset();
}

void PidAdvanced::setDt(float dt)               { _dt = dt; }
void PidAdvanced::setDesired(float desired)     { _desired = desired; }
void PidAdvanced::setIntegralLimit(float limit) { _integralLimit = limit; }
void PidAdvanced::setOutputLimit(float limit)   { _outputLimit = limit; }

// ─── reset ────────────────────────────────────────────────────────────────────
// Must be called on every arm/disarm cycle.
// Without this, accumulated integral from a previous flight biases takeoff.
void PidAdvanced::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
}

// ─── update ───────────────────────────────────────────────────────────────────
// Standard PID — D-term computed from finite difference of error.
// Use this only if gyro rate is unavailable for this axis.
float PidAdvanced::update(float measured) {
    float error = _desired - measured;

    // P-term: proportional to current error
    float p = _kp * error;

    // I-term: accumulated error over time — clamped to prevent windup
    _integral += error * _dt;
    _integral  = constrain(_integral, -_integralLimit, _integralLimit);
    float i    = _ki * _integral;

    // D-term: rate of change of error (noisy — prefer updateWithRate if possible)
    float derivative = (error - _prevError) / _dt;
    float d          = _kd * derivative;
    _prevError       = error;

    // Sum and clamp output to duty-cycle safe range
    float output = p + i + d;
    return constrain(output, -_outputLimit, _outputLimit);
}

// ─── updateWithRate ───────────────────────────────────────────────────────────
// Preferred update path — uses measured gyro rate as the D-term directly.
// This avoids differentiating the angle (which amplifies quantization noise).
// measuredRate: gyro reading for this axis in deg/s (gx=roll, gy=pitch, gz=yaw)
float PidAdvanced::updateWithRate(float measured, float measuredRate) {
    float error = _desired - measured;

    // P-term
    float p = _kp * error;

    // I-term with anti-windup clamp
    _integral += error * _dt;
    _integral  = constrain(_integral, -_integralLimit, _integralLimit);
    float i    = _ki * _integral;

    // D-term: directly from gyro rate (desired rate = 0 for angle-hold)
    // We negate because positive rate means the error is shrinking
    float d = -_kd * measuredRate;

    float output = p + i + d;
    return constrain(output, -_outputLimit, _outputLimit);
}