#include "stabilizer.h"

// ─── init ─────────────────────────────────────────────────────────────────────
// Called once in main() after IMU calibration completes.
// Seeds the fusion filter at level (0,0,0) — drone is flat on the ground.
// Resets PID integral state so no stale error biases the first hover.
void Stabilizer::init(FlightSystem& fs) {
    _fs = &fs;

    // Seed fusion with level orientation — drone must be flat during calibration
    _fusion.init(0.0f, 0.0f, 0.0f);

    // Initialize PID at 500Hz default — dt gets recomputed live every update()
    _pid.init(0.002f);
    _pid.reset(); // Clear any stale integral state

    _lastUs = micros();

    Serial.println("[STAB] Stabilizer initialized.");
}

// ─── update ───────────────────────────────────────────────────────────────────
// The main control pulse — called every loop iteration in main().
// Pipeline: IMU raw → fusion → safety check → PID → mix → MotorMix
MotorMix Stabilizer::update(const Setpoint& setpoint, const IMUData& imu) {

    // ── 1. Compute live dt ────────────────────────────────────────────────────
    // Always measure actual elapsed time — never assume a fixed loop rate.
    // Jitter from I2C, serial prints, or flash writes would corrupt integration.
    unsigned long nowUs = micros();
    _dt = (nowUs - _lastUs) / 1000000.0f;
    _lastUs = nowUs;

    // Guard against absurd dt values (first frame, CPU stall, micros() overflow)
    // Clamp to a reasonable range: 0.5ms–50ms (2000Hz–20Hz)
    if (_dt < 0.0005f || _dt > 0.05f) {
        _dt = 0.002f; // Fall back to 500Hz default — skip this frame's integration
    }

    // ── 2. Feed raw IMU into sensor fusion ────────────────────────────────────
    // SensFusion6 runs the complementary filter and returns fused Euler angles.
    // imu carries calibrated accel (G) and gyro (deg/s) — no angles from imu.cpp.
    _fusion.update(imu.gx, imu.gy, imu.gz,
                   imu.ax, imu.ay, imu.az,
                   _dt);
    AttitudeDeg att = _fusion.attitude();

    // ── 3. Safety gate ────────────────────────────────────────────────────────
    // FlightSystem checks tilt limits, IMU health, phase, and elapsed time.
    // If anything is wrong, we return a zeroed mix immediately — no PID runs.
    // main() will also see shouldCutPower() and skip the motor write.
    if (_fs->shouldCutPower()) {
        MotorMix zero = {0, 0, 0, 0};
        return zero;
    }

    // ── 4. IMU health check ───────────────────────────────────────────────────
    // imu.isHealthy catches I2C failures and accel magnitude anomalies.
    // Trigger emergency stop — this cuts motors inside FlightSystem immediately.
    if (!imu.isHealthy) {
        return emergencyStop("IMU unhealthy in stabilizer");
    }

    // ── 5. Run PID ────────────────────────────────────────────────────────────
    // Build the rate struct from raw gyro — used for D-term (no noise from differencing)
    AttitudeRateDps rate;
    rate.roll  = imu.gx; // deg/s — matches roll  axis
    rate.pitch = imu.gy; // deg/s — matches pitch axis
    rate.yaw   = imu.gz; // deg/s — matches yaw   axis

    // Update dt in PID so integral and derivative scale correctly this frame
    _pid.setDt(_dt);

    // Run three-axis PID — returns signed duty-cycle corrections
    ControlOutput correction = _pid.update(setpoint, att, rate);

    // ── 6. Mix thrust + corrections into per-motor duty cycles ────────────────
    // HOVER_THRUST is the constant upward force for this mission.
    // PID corrections are signed deltas added on top by PowerDistribution.
    MotorMix mix = _mixer.mixX(HOVER_THRUST, correction);

    return mix;
}

// ─── emergencyStop ────────────────────────────────────────────────────────────
// Hard stop — zeroes the returned mix AND directly cuts motor hardware.
// Belt-and-suspenders: even if main() fails to act on the zeroed mix,
// the LEDC outputs are already at 0 from stopAllMotors().
MotorMix Stabilizer::emergencyStop(const char* reason) {
    // Cut hardware immediately — don't wait for main() to write the mix
    stopAllMotors(); // motors.cpp — sets all LEDC duty to 0, clears armed flag

    // Also tell FlightSystem so it transitions to HALTED
    if (_fs) _fs->triggerEmergencyStop(reason);

    // Reset PID so if (somehow) the system recovers, no stale integral remains
    _pid.reset();

    MotorMix zero = {0, 0, 0, 0};
    return zero;
}

// ─── getAttitude ──────────────────────────────────────────────────────────────
// Exposes the latest fused attitude for serial telemetry.
// Call from main() to print roll/pitch/yaw without re-running fusion.
AttitudeDeg Stabilizer::getAttitude() const {
    return _fusion.attitude();
}