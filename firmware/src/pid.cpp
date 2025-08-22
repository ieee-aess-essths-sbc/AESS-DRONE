#include "pid.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float minOutput, float maxOutput) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _minOutput = minOutput;
    _maxOutput = maxOutput;
    _integral = 0;
    _lastError = 0;
}

float PID::compute(float target, float current, float dt) {
    if (dt <= 0) dt = 0.02; // Default to 20ms if invalid
    
    float error = target - current;
    
    // Proportional term
    float p = _kp * error;
    
    // Integral term with anti-windup
    _integral += error * dt;
    
    // Constrain integral to prevent windup
    float i = _ki * _integral;
    if (i > _maxOutput) {
        i = _maxOutput;
        _integral = i / _ki; // Back-calculate to constrain integral
    } else if (i < _minOutput) {
        i = _minOutput;
        _integral = i / _ki;
    }
    
    // Derivative term
    float derivative = (error - _lastError) / dt;
    float d = _kd * derivative;
    _lastError = error;
    
    // Calculate total output
    float output = p + i + d;
    
    // Constrain output
    if (output > _maxOutput) output = _maxOutput;
    if (output < _minOutput) output = _minOutput;
    
    return output;
}

void PID::reset() {
    _integral = 0;
    _lastError = 0;
}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
