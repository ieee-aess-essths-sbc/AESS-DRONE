#include "motors.h"

// ─── Module-Private State ─────────────────────────────────────────────────────
static bool motorsArmed = false; // Safety gate — outputs blocked until armed

// Pin lookup table — index 0 = motor 1, index 3 = motor 4
static const uint8_t motorPins[4] = {
    MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN
};

// ─── Internal Helper ──────────────────────────────────────────────────────────
// Writes a duty cycle directly to one MOSFET gate via LEDC.
// Clamps to MOTOR_MAX so no code path can accidentally over-drive the motors.
static void writeMotor(uint8_t index, uint8_t duty) {
    // Hard cap — enforced at the lowest level so every code path respects it
    if (duty > MOTOR_MAX) duty = MOTOR_MAX;
    ledcWrite(motorPins[index], duty);
}

// ─── initMotors ───────────────────────────────────────────────────────────────
// Attaches all motor pins to the LEDC PWM engine at 20kHz / 8-bit.
// Motors stay at 0 duty until armMotors() is called.
void initMotors() {
    for (int i = 0; i < 4; i++) {
        // ledcAttach(pin, freq, resolution) — new ESP32 Arduino core API (≥3.x)
        ledcAttach(motorPins[i], PWM_FREQ, PWM_RES);
    }
    stopAllMotors(); // Ensure all gates are LOW on boot
    motorsArmed = false;
    Serial.println("[MOTORS] Initialized — all outputs at 0, disarmed.");
}

// ─── armMotors ────────────────────────────────────────────────────────────────
// Soft ramp from 0 → MOTOR_ARM_TARGET over ~600ms.
// Prevents the inrush current spike that can reset or brown-out the ESP32.
// After ramp the arm flag is set — setMotorSpeeds() becomes active.
void armMotors() {
    Serial.println("[MOTORS] Arming — soft ramp starting...");

    // Gradually increase duty cycle from 0 to ARM_TARGET
    for (uint8_t duty = 0; duty <= MOTOR_ARM_TARGET; duty++) {
        for (int i = 0; i < 4; i++) writeMotor(i, duty);
        delay(10); // 10ms per step × ~40 steps = ~400ms ramp
    }

    motorsArmed = true;
    Serial.printf("[MOTORS] Armed — running at idle duty %d/255.\n", MOTOR_ARM_TARGET);
}

// ─── stopAllMotors ────────────────────────────────────────────────────────────
// Immediately sets all MOSFET gates to 0 (fully off).
// Called by stabilizer on disarm, crash detect, or timed mission end.
// Also clears the armed flag so setMotorSpeeds() stops working.
void stopAllMotors() {
    for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], 0);
    motorsArmed = false;
    Serial.println("[MOTORS] STOPPED — all motors off, disarmed.");
}

// ─── setMotorSpeed ────────────────────────────────────────────────────────────
// Sets a single motor (motorNum 1–4) to a given speed (0–255).
// Will silently do nothing if motors are not armed — safety gate.
void setMotorSpeed(uint8_t motorNum, uint8_t speed) {
    if (!motorsArmed) return; // Safety gate — ignore commands when disarmed
    if (motorNum < 1 || motorNum > 4) return; // Bounds check

    // Enforce minimum spin: if caller sends >0 but below MOTOR_MIN,
    // clamp up to MOTOR_MIN so motors don't stall under load
    if (speed > 0 && speed < MOTOR_MIN) speed = MOTOR_MIN;

    writeMotor(motorNum - 1, speed); // Convert 1-based to 0-based index
}

// ─── setMotorSpeeds ───────────────────────────────────────────────────────────
// Convenience function to update all four motors in one call.
// power_distrib.cpp calls this every control loop iteration.
void setMotorSpeeds(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
    setMotorSpeed(1, m1);
    setMotorSpeed(2, m2);
    setMotorSpeed(3, m3);
    setMotorSpeed(4, m4);
}

// ─── motorsAreArmed ───────────────────────────────────────────────────────────
// Lets other modules (stabilizer, main) query arm state without
// accessing the private flag directly.
bool motorsAreArmed() {
    return motorsArmed;
}