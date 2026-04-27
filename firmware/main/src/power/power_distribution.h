#ifndef POWER_DISTRIB_H
#define POWER_DISTRIB_H

#include <Arduino.h>
#include "flight_types.h"
#include "motors.h"  // For MOTOR_MIN, MOTOR_MAX constants

// ─── MotorMix ─────────────────────────────────────────────────────────────────
// Output of the mixing function — one duty cycle per motor (0–255).
// Passed directly to setMotorSpeeds() in motors.cpp.
struct MotorMix {
    uint8_t m1; // Front-Left  (CCW)
    uint8_t m2; // Front-Right (CW)
    uint8_t m3; // Rear-Right  (CCW)
    uint8_t m4; // Rear-Left   (CW)
};

// ─── PowerDistribution ────────────────────────────────────────────────────────
// Stateless mixer — takes PID correction outputs + base thrust,
// applies quad-X mixing algebra, then scales to safe 0–255 range.
class PowerDistribution {
public:
    // mixX: Standard quad-X mixing for your 8520 brushed layout.
    // thrust : base duty (0–255) from main — the hover set-point
    // c      : PID roll/pitch/yaw corrections (signed, in duty-cycle units)
    MotorMix mixX(uint8_t thrust, const ControlOutput& c) const;
};

#endif // POWER_DISTRIB_H