#ifndef STABILIZER_H
#define STABILIZER_H

#include "imu.h"
#include "sensfusion6.h"
#include "controller_pid.h"
#include "power_distrib.h"
#include "flight_system.h"
#include "flight_types.h"
#include "motors.h"

// ─── Stabilizer ───────────────────────────────────────────────────────────────
// The main control loop hub. Every iteration it:
//   1. Reads dt from micros() (live, not fixed)
//   2. Feeds raw IMU data into SensFusion6 → fused attitude
//   3. Checks FlightSystem safety gate
//   4. Runs ControllerPid → signed corrections
//   5. Passes thrust + corrections to PowerDistribution → MotorMix
//   6. Returns MotorMix to main() which writes it to motors
//
// Stabilizer does NOT write to motors directly — that stays in main()
// so there is always one clear place where motor writes happen.

class Stabilizer {
public:
    // init: call once after IMU calibration and before arming.
    // fs: reference to the shared FlightSystem state machine.
    void init(FlightSystem& fs);

    // update: call every loop iteration.
    // Returns the motor mix to apply — main() writes it to setMotorSpeeds().
    // If shouldCutPower() is true, returns a zeroed mix without running PID.
    MotorMix update(const Setpoint& setpoint, const IMUData& imu);

    // emergencyStop: immediately zeroes mix and cuts motors.
    // Call this from any ISR or watchdog that detects a hard fault.
    MotorMix emergencyStop(const char* reason);

    // getAttitude: exposes fused attitude for serial telemetry / debugging.
    AttitudeDeg getAttitude() const;

private:
    SensFusion6      _fusion;       // Complementary filter — owns angle estimation
    ControllerPid    _pid;          // Three-axis PID — owns correction calculation
    PowerDistribution _mixer;       // Quad-X mixer — owns motor output mapping
    FlightSystem*    _fs = nullptr; // Non-owning pointer to shared safety manager

    unsigned long _lastUs  = 0;     // micros() timestamp of last update() call
    float         _dt      = 0.002f;// Computed live each frame (seconds)

    // Base hover thrust in duty-cycle units (0–255).
    // This is the constant upward force — PID corrections are added on top.
    // Tune this until the drone just lifts off, then add ~10% headroom.
    // For 8520 motors at 3.7V on a ~35g frame, ~110–130 is a typical starting point.
    static constexpr uint8_t HOVER_THRUST = 120;
};

#endif // STABILIZER_H
