#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include "flight_types.h"
#include "pid_advanced.h"

// ─── ControllerPid ────────────────────────────────────────────────────────────
// Runs three independent PID loops: roll angle, pitch angle, yaw rate.
// Takes current IMU state, compares to setpoint, returns signed corrections.
// Thrust is NOT handled here — it belongs to main/stabilizer.
class ControllerPid {
public:
    void init(float dt);           // Set gains, limits, initial dt
    void setDt(float dt);          // Update dt if loop timing changes
    void reset();                  // Wipe integral state (call on arm/disarm)

    // update: pass current attitude + raw gyro rates for D-term
    // Returns signed duty-cycle corrections for roll, pitch, yaw
    ControlOutput update(const Setpoint&        setpoint,
                         const AttitudeDeg&     attitude,
                         const AttitudeRateDps& rate);

private:
    PidAdvanced _attRoll;   // Roll  angle → duty correction
    PidAdvanced _attPitch;  // Pitch angle → duty correction
    PidAdvanced _yawRate;   // Yaw rate    → duty correction (rate control, not angle)

    float _dt = 0.002f; // Default 500Hz — matches IMU sample rate
};

#endif // CONTROLLER_PID_H