#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// ─── Motor GPIO Pins (ESP32-S3 Zero → your MOSFET gate pins) ─────────────────
// Change these to match your PCB layout
#define MOTOR1_PIN  5   // Front-Left  (CCW)
#define MOTOR2_PIN  6   // Front-Right (CW)
#define MOTOR3_PIN  7   // Rear-Right  (CCW)
#define MOTOR4_PIN  8   // Rear-Left   (CW)

// ─── PWM Configuration ───────────────────────────────────────────────────────
// 20kHz is above human hearing — no whine, lower switching losses than 50Hz
// 8-bit resolution = 0–255 duty cycle steps (plenty for brushed motors)
#define PWM_FREQ        20000   // Hz
#define PWM_RES         8       // bits → 0 to 255
#define PWM_MAX_DUTY    255     // 100% duty = full throttle

// ─── Throttle Limits (0–255 scale) ───────────────────────────────────────────
// MOTOR_MIN: lowest duty that actually spins the 8520 reliably under load
// MOTOR_MAX: capped for safety — 8520 at 3.7V, short hover mission, no headroom needed
// MOTOR_ARM: gentle spin-up target during arming ramp
#define MOTOR_MIN       30      // ~12% duty — stall-free minimum spin
#define MOTOR_MAX       200     // ~78% duty — enough to hover, limits heat & current
#define MOTOR_ARM_TARGET 40     // Ramp-to value during soft arm sequence

// ─── Public API ──────────────────────────────────────────────────────────────
void initMotors();                                      // Configure LEDC PWM channels
void armMotors();                                       // Soft ramp-up to confirm spin
void stopAllMotors();                                   // Instantly cut all outputs to 0
void setMotorSpeed(uint8_t motorNum, uint8_t speed);   // Set one motor (1–4), speed 0–255
void setMotorSpeeds(uint8_t m1, uint8_t m2,
                    uint8_t m3, uint8_t m4);           // Set all four at once
bool motorsAreArmed();                                  // Query arm state

#endif // MOTORS_H