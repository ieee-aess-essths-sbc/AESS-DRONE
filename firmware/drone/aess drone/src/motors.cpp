#include "motors.h"
#include <Arduino.h>
#include "motors.h"


const int motorPins[4] = {4, 5, 6, 7}; //change later based on what weve got

void initMotors() {
    for (int i = 0; i < 4; i++) {
        ledcAttachPin(motorPins[i], i);
        ledcSetup(i, 50, 16); // 50Hz for ESCs
    }
}

void setMotorSpeeds(float throttle, float roll, float pitch, float yaw) {
    float m1 = throttle + pitch + roll - yaw;
    float m2 = throttle + pitch - roll + yaw;
    float m3 = throttle - pitch + roll + yaw;
    float m4 = throttle - pitch - roll - yaw;

    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);

    ledcWrite(0, m1);
    ledcWrite(1, m2);
    ledcWrite(2, m3);
    ledcWrite(3, m4);
}
