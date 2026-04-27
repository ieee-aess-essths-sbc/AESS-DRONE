#include "flight_system.h"
#include "motors.h"

// ─── Tilt Safety Limit ────────────────────────────────────────────────────────
// If the drone exceeds this angle in any direction it's considered unrecoverable.
// Triggers immediate emergency stop. Must match or be tighter than imu.cpp's limit.
#define MAX_TILT_DEG  55.0f

// ─── init ─────────────────────────────────────────────────────────────────────
// Called once in main setup(). Configures timing parameters and starts in BOOT.
// armDelayMs   : how long after calibration to wait before spinning motors
// hoverDuration: how long to actively hover before auto-disarm
void FlightSystem::init(unsigned long armDelayMs, unsigned long hoverDurationMs) {
    _armDelayMs      = armDelayMs;
    _hoverDurationMs = hoverDurationMs;
    _emergencyStop   = false;
    _stopReason      = nullptr;
    _enterPhase(FlightPhase::BOOT);
    Serial.println("[FSM] FlightSystem initialized — phase: BOOT");
}

// ─── _enterPhase ──────────────────────────────────────────────────────────────
// Single point of truth for all phase transitions.
// Logs every transition so the serial monitor shows a clear mission timeline.
void FlightSystem::_enterPhase(FlightPhase next) {
    _phase        = next;
    _phaseStartMs = millis();
    Serial.printf("[FSM] → Phase: %s  (t=%lums)\n", getPhaseName(), _phaseStartMs);
}

// ─── notifyCalibrationDone ────────────────────────────────────────────────────
// Called by main() after initIMU() and calibrateIMU() complete.
// Transitions BOOT → READY and starts the pre-arm countdown.
void FlightSystem::notifyCalibrationDone() {
    if (_phase != FlightPhase::BOOT) return; // Guard: only valid from BOOT
    _enterPhase(FlightPhase::READY);
    Serial.printf("[FSM] Arming in %.1f seconds — keep drone flat.\n",
                  _armDelayMs / 1000.0f);
}

// ─── notifyArmingComplete ─────────────────────────────────────────────────────
// Called by main() after armMotors() soft ramp finishes.
// Transitions ARMING → HOVERING and starts the hover countdown.
void FlightSystem::notifyArmingComplete() {
    if (_phase != FlightPhase::ARMING) return; // Guard: only valid from ARMING
    _enterPhase(FlightPhase::HOVERING);
    Serial.printf("[FSM] Hovering for %.1f seconds then auto-disarm.\n",
                  _hoverDurationMs / 1000.0f);
}

// ─── triggerEmergencyStop ─────────────────────────────────────────────────────
// Hard stop — callable from anywhere (IMU health check, tilt limit, main loop).
// Cuts motors immediately and moves to HALTED. Cannot be reversed without reboot.
void FlightSystem::triggerEmergencyStop(const char* reason) {
    if (_phase == FlightPhase::HALTED) return; // Already stopped, don't spam serial
    _emergencyStop = true;
    _stopReason    = reason;
    stopAllMotors(); // motors.h — immediate gate LOW on all 4 channels
    _enterPhase(FlightPhase::HALTED);
    Serial.printf("[FSM] *** EMERGENCY STOP: %s ***\n", reason);
}

// ─── update ───────────────────────────────────────────────────────────────────
// Called every control loop iteration (~500Hz).
// Checks safety conditions and advances the state machine by time.
void FlightSystem::update(const SafetyInputs& s) {

    // ── Always-on safety checks (run in every phase) ─────────────────────────

    // IMU failure — if sensor goes offline mid-flight, cut immediately
    if (!s.imuHealthy && _phase == FlightPhase::HOVERING) {
        triggerEmergencyStop("IMU unhealthy");
        return;
    }

    // Tilt limit — drone is flipping or has crashed
    if ((fabsf(s.rollDeg)  > MAX_TILT_DEG ||
         fabsf(s.pitchDeg) > MAX_TILT_DEG) &&
         _phase == FlightPhase::HOVERING) {
        triggerEmergencyStop("Tilt limit exceeded");
        return;
    }

    // ── Phase-specific timed transitions ─────────────────────────────────────
    unsigned long elapsed = millis() - _phaseStartMs;

    switch (_phase) {

        case FlightPhase::BOOT:
            // Waiting for notifyCalibrationDone() — nothing to do here
            break;

        case FlightPhase::READY:
            // Count down the pre-arm delay then move to ARMING
            // main() will call armMotors() when it sees phase == ARMING
            if (elapsed >= _armDelayMs) {
                _enterPhase(FlightPhase::ARMING);
            }
            break;

        case FlightPhase::ARMING:
            // Waiting for main() to call armMotors() + notifyArmingComplete()
            // Soft ramp takes ~400ms — handled in motors.cpp
            break;

        case FlightPhase::HOVERING:
            // Auto-disarm when hover duration expires — normal mission end
            if (elapsed >= _hoverDurationMs) {
                Serial.println("[FSM] Hover time complete — initiating disarm.");
                _enterPhase(FlightPhase::DISARMING);
                stopAllMotors(); // motors.cpp clears armed flag + zeroes gates
            }
            break;

        case FlightPhase::DISARMING:
            // Motors are already stopped — wait one second then move to HALTED
            // This gives the frame time to fully spin down before flagging complete
            if (elapsed >= 1000) {
                _enterPhase(FlightPhase::HALTED);
                Serial.println("[FSM] Mission complete. Safe to handle.");
            }
            break;

        case FlightPhase::HALTED:
            // Terminal state — nothing runs, loop just idles
            break;
    }
}

// ─── shouldCutPower ───────────────────────────────────────────────────────────
// The stabilizer calls this before every motor write.
// Returns true in every state except HOVERING — motors only run during hover.
bool FlightSystem::shouldCutPower() const {
    return _emergencyStop || (_phase != FlightPhase::HOVERING);
}

// ─── isHovering ───────────────────────────────────────────────────────────────
bool FlightSystem::isHovering() const {
    return _phase == FlightPhase::HOVERING;
}

// ─── isHalted ─────────────────────────────────────────────────────────────────
bool FlightSystem::isHalted() const {
    return _phase == FlightPhase::HALTED;
}

// ─── getPhase / getPhaseName ──────────────────────────────────────────────────
FlightPhase FlightSystem::getPhase() const {
    return _phase;
}

const char* FlightSystem::getPhaseName() const {
    switch (_phase) {
        case FlightPhase::BOOT:      return "BOOT";
        case FlightPhase::READY:     return "READY";
        case FlightPhase::ARMING:    return "ARMING";
        case FlightPhase::HOVERING:  return "HOVERING";
        case FlightPhase::DISARMING: return "DISARMING";
        case FlightPhase::HALTED:    return "HALTED";
        default:                     return "UNKNOWN";
    }
}