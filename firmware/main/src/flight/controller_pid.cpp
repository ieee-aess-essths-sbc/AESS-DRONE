#include "controller_pid.h"

// ─── PID Gains ────────────────────────────────────────────────────────────────
// Tuned for 8520 brushed motors at 3.7V — low inertia, fast response.
// Output units are duty-cycle deltas (0–255 scale).
//
// Roll / Pitch:
//   Kp = 4.5  — firm correction without oscillation on light brushed frame
//   Ki = 1.5  — slow integrator, corrects steady lean without windup
//   Kd = 0.08 — light damping via gyro rate (not finite difference)
//
// Yaw (rate control — not angle):
//   Kp = 5.0  — yaw is sluggish on brushed quads, needs more P
//   Ki = 0.8  — light integrator
//   Kd = 0.0  — no D on yaw rate (gyro noise + no real benefit)
//
// ⚠ These are starting values — always tune on a test rig before free flight.
// Increase Kp until oscillation, then back off 30%. Add Kd to damp.

#define ROLL_KP   4.5f
#define ROLL_KI   1.5f
#define ROLL_KD   0.08f

#define PITCH_KP  4.5f
#define PITCH_KI  1.5f
#define PITCH_KD  0.08f

#define YAW_KP    5.0f
#define YAW_KI    0.8f
#define YAW_KD    0.0f

// ─── init ─────────────────────────────────────────────────────────────────────
void ControllerPid::init(float dt) {
    _dt = dt;

    _attRoll.init (ROLL_KP,  ROLL_KI,  ROLL_KD,  _dt);
    _attPitch.init(PITCH_KP, PITCH_KI, PITCH_KD, _dt);
    _yawRate.init (YAW_KP,   YAW_KI,   YAW_KD,   _dt);

    // ── Integral Limits (anti-windup) ─────────────────────────────────────────
    // Cap how much the integrator can accumulate.
    // 15 duty units is enough to correct a steady lean without runaway.
    _attRoll.setIntegralLimit (15.0f);
    _attPitch.setIntegralLimit(15.0f);
    _yawRate.setIntegralLimit (10.0f);

    // ── Output Limits ─────────────────────────────────────────────────────────
    // Max correction any single axis can apply in duty-cycle units (0–255 scale).
    // 80 = ~31% authority — enough to correct attitude, not enough to flip a motor
    _attRoll.setOutputLimit (80.0f);
    _attPitch.setOutputLimit(80.0f);
    _yawRate.setOutputLimit (60.0f); // Yaw gets less authority — less critical

    Serial.println("[PID] Controller initialized.");
}

// ─── setDt ────────────────────────────────────────────────────────────────────
void ControllerPid::setDt(float dt) {
    _dt = dt;
    _attRoll.setDt(dt);
    _attPitch.setDt(dt);
    _yawRate.setDt(dt);
}

// ─── reset ────────────────────────────────────────────────────────────────────
// Call this every time the drone arms or disarms.
// Stale integral from a previous flight will bias the drone immediately on takeoff.
void ControllerPid::reset() {
    _attRoll.reset();
    _attPitch.reset();
    _yawRate.reset();
    Serial.println("[PID] Integral state cleared.");
}

// ─── update ───────────────────────────────────────────────────────────────────
// Called every control loop iteration (~500Hz).
// Uses updateWithRate() for roll and pitch so the D-term reads directly
// from gyro (clean) rather than differentiating the fused angle (noisy).
// Yaw uses rate control — desired yaw rate = 0 means hold heading.
ControlOutput ControllerPid::update(const Setpoint&        setpoint,
                                     const AttitudeDeg&     attitude,
                                     const AttitudeRateDps& rate) {
    ControlOutput out;

    // Set targets for each axis
    _attRoll.setDesired (setpoint.attitudeDeg.roll);
    _attPitch.setDesired(setpoint.attitudeDeg.pitch);
    _yawRate.setDesired (setpoint.attitudeRateDps.yaw); // Target yaw rate = 0 = hold

    // Roll: angle error corrected, D-term from gyro X rate
    out.roll  = _attRoll.updateWithRate (attitude.roll,  rate.roll);

    // Pitch: angle error corrected, D-term from gyro Y rate
    out.pitch = _attPitch.updateWithRate(attitude.pitch, rate.pitch);

    // Yaw: pure rate control — error is (desired rate - actual rate)
    // updateWithRate still works here: desired=0, measured=current heading, rate=gz
    out.yaw   = _yawRate.updateWithRate (attitude.yaw,   rate.yaw);

    return out;
}