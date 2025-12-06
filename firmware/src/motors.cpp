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
    setMotorSpeeds(1000, 1000, 1000, 1000);
}

void setMotorSpeeds(int m1, int m2, int m3, int m4) {
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);
    uint16_t duty1 = map(m1, 1000, 2000, 3276, 6553);
    uint16_t duty2 = map(m2, 1000, 2000, 3276, 6553);
    uint16_t duty3 = map(m3, 1000, 2000, 3276, 6553);
    uint16_t duty4 = map(m4, 1000, 2000, 3276, 6553);
    ledcWrite(0, duty1);
    ledcWrite(1, duty2);
    ledcWrite(2, duty3);
    ledcWrite(3, duty4);
}