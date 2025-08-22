#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(float kp, float ki, float kd, float minOutput, float maxOutput);
    float compute(float target, float current, float dt);
    void reset();
    void setGains(float kp, float ki, float kd);
    
  private:
    float _kp, _ki, _kd;
    float _integral = 0;
    float _lastError = 0;
    float _minOutput, _maxOutput;
};

#endif