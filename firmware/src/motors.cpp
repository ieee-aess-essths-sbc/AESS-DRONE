#include "motors.h"

void initMotors() {
    // Configure motor pins for PWM
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcSetup(1, PWM_FREQ, PWM_RES);
    ledcSetup(2, PWM_FREQ, PWM_RES);
    ledcSetup(3, PWM_FREQ, PWM_RES);

    ledcAttachPin(MOTOR1, 0);
    ledcAttachPin(MOTOR2, 1);
    ledcAttachPin(MOTOR3, 2);
    ledcAttachPin(MOTOR4, 3);
}

void setMotorSpeeds(float m1, float m2, float m3, float m4) {
    m1 = constrain(m1, 0, 255);
    m2 = constrain(m2, 0, 255);
    m3 = constrain(m3, 0, 255);
    m4 = constrain(m4, 0, 255);

    ledcWrite(0, m1);
    ledcWrite(1, m2);
    ledcWrite(2, m3);
    ledcWrite(3, m4);
}