#include "power_distrib.h"

// ─── Quad-X Motor Layout (top-down view) ──────────────────────────────────────
//
//       FRONT
//   M1(CCW)  M2(CW)
//      \      /
//       [BODY]
//      /      \
//   M4(CW)  M3(CCW)
//       REAR
//
// CCW motors: M1 (Front-Left), M3 (Rear-Right)
// CW  motors: M2 (Front-Right), M4 (Rear-Left)
//
// Mixing sign table (standard quad-X):
// ─────────────────────────────────────────────────
//        │ Thrust │ Roll │ Pitch │ Yaw
// ───────┼────────┼──────┼───────┼─────
//   M1   │   +    │  +   │   +   │  +   (CCW → yaw +)
//   M2   │   +    │  -   │   +   │  -   (CW  → yaw -)
//   M3   │   +    │  -   │   -   │  +   (CCW → yaw +)
//   M4   │   +    │  +   │   -   │  -   (CW  → yaw -)
// ─────────────────────────────────────────────────
// Roll+  = right side up  → M1/M4 increase, M2/M3 decrease
// Pitch+ = nose up        → M1/M2 increase, M3/M4 decrease
// Yaw+   = rotate CCW     → CCW motors faster, CW motors slower

MotorMix PowerDistribution::mixX(uint8_t thrust, const ControlOutput& c) const {
    MotorMix mix;

    // ── Hard zero when thrust is 0 (disarmed / pre-flight) ───────────────────
    // Do NOT apply MOTOR_MIN floor here — motors should be fully off
    // when the stabilizer isn't commanding flight.
    if (thrust == 0) {
        mix.m1 = mix.m2 = mix.m3 = mix.m4 = 0;
        return mix;
    }

    // ── Apply Quad-X Mixing ───────────────────────────────────────────────────
    // All values are in float to preserve fractional corrections before rounding.
    // c.roll / c.pitch / c.yaw come from the PID controller in duty-cycle units.
    float m1 = (float)thrust + c.roll + c.pitch + c.yaw;  // Front-Left  CCW
    float m2 = (float)thrust - c.roll + c.pitch - c.yaw;  // Front-Right CW
    float m3 = (float)thrust - c.roll - c.pitch + c.yaw;  // Rear-Right  CCW
    float m4 = (float)thrust + c.roll - c.pitch - c.yaw;  // Rear-Left   CW

    // ── Priority Scaling (preserve mix ratios instead of hard-clamping) ───────
    // Hard clamping (constrain) breaks the attitude: if M1 clips at MOTOR_MAX
    // but M3 doesn't, the drone tilts because the intended differential is lost.
    // Instead: find the highest output — if it exceeds MOTOR_MAX, scale ALL
    // four motors down by the same factor so the ratios are preserved.
    float maxOut = max({m1, m2, m3, m4});
    if (maxOut > MOTOR_MAX) {
        float scale = (float)MOTOR_MAX / maxOut;
        m1 *= scale;
        m2 *= scale;
        m3 *= scale;
        m4 *= scale;
    }

    // ── Apply Idle Floor (during active flight only) ──────────────────────────
    // Prevent any motor from dropping below MOTOR_MIN while hovering.
    // A brushed 8520 that stalls mid-flight won't restart fast enough to recover.
    // We only apply this floor AFTER scaling, so it doesn't distort the mix.
    auto applyFloor = [](float val) -> uint8_t {
        if (val < MOTOR_MIN) val = MOTOR_MIN;
        if (val > MOTOR_MAX) val = MOTOR_MAX; // Final safety ceiling
        return (uint8_t)val;
    };

    mix.m1 = applyFloor(m1);
    mix.m2 = applyFloor(m2);
    mix.m3 = applyFloor(m3);
    mix.m4 = applyFloor(m4);

    return mix;
}