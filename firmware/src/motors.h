// include/motors.h
#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// Motor pin assignments (update to match your hardware)
#define MOTOR1 11
#define MOTOR2 5
#define MOTOR3 2
#define MOTOR4 13

#define PWM_FREQ 5000     // 5 kHz PWM frequency
#define PWM_RES 8         // 8-bit resolution (0-255)

void initMotors();
void setMotorSpeeds(float m1, float m2, float m3, float m4);

#endif
