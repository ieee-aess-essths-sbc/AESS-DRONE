#ifndef FLIGHT_SYSTEM_H
#define FLIGHT_SYSTEM_H

#include <Arduino.h>

// ─── Flight Phase State Machine ───────────────────────────────────────────────
// The drone progresses through these states in order.
// No state can be skipped — each transition is explicitly guarded.
enum class FlightPhase {
    BOOT,           // System just powered on — IMU calibrating
    READY,          // Calibration done — waiting for arm delay
    ARMING,         // Soft motor ramp in progress
    HOVERING,       // Active flight — PID loop running
    DISARMING,      // Timed mission complete — motors ramping down
    HALTED          // All motors off — terminal state until reboot
};

// ─── FlightSystem ─────────────────────────────────────────────────────────────
// Central safety manager. Owns the state machine and all disarm triggers.
// The stabilizer loop calls update() every iteration and checks
// shouldCutPower() before writing anything to motors.
class FlightSystem {
public:
    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void init(unsigned long armDelayMs, unsigned long hoverDurationMs);
    void update(const struct SafetyInputs& inputs); // Call every control loop
    void triggerEmergencyStop(const char* reason);  // Immediate motor cut

    // ── State Queries (used by stabilizer + main) ─────────────────────────────
    bool shouldCutPower()  const; // true → stabilizer must write 0 to all motors
    bool isHovering()      const; // true → PID loop should run
    bool isHalted()        const; // true → nothing can restart without reboot
    FlightPhase getPhase() const;
    const char* getPhaseName() const;

    // ── Phase Notifications (called by main/motors when transitions complete) ──
    void notifyCalibrationDone(); // BOOT → READY
    void notifyArmingComplete();  // ARMING → HOVERING

private:
    FlightPhase _phase = FlightPhase::BOOT;

    // Timing
    unsigned long _armDelayMs      = 3000; // Wait after calibration before arming
    unsigned long _hoverDurationMs = 15000;// How long to actively hover
    unsigned long _phaseStartMs    = 0;    // When current phase began

    // Safety
    bool _emergencyStop = false;
    const char* _stopReason = nullptr;

    // Internal transitions
    void _enterPhase(FlightPhase next);
};

// ─── SafetyInputs ─────────────────────────────────────────────────────────────
// Snapshot of all safety-relevant sensor data passed into update().
// Keeping it as a struct means adding new checks never changes the signature.
struct SafetyInputs {
    float rollDeg;    // Current roll  from IMU (degrees)
    float pitchDeg;   // Current pitch from IMU (degrees)
    bool  imuHealthy; // IMU I2C read succeeded and dt was valid
};

#endif // FLIGHT_SYSTEM_H