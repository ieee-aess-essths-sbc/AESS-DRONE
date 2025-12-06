#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// Motor pin assignments 
#define MOTOR1 1
#define MOTOR2 5
#define MOTOR3 2
#define MOTOR4 13

#define PWM_FREQ 50     // 50 Hz PWM frequency
#define PWM_RES 16         // 16-bit resolution 

void initMotors();
void setMotorSpeeds(int m1, int m2, int m3, int m4);

#endif