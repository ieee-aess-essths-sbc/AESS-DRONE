#include "pid.h"

PID::PID(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0;
    _lastError = 0;
}

float PID::compute(float target, float current) {
    float error = target - current;
    float derivative = error - _lastError;
    _integral += error;
    _lastError = error;
    return _kp * error + _ki * _integral + _kd * derivative;
}
