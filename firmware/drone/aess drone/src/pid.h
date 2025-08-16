#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(float kp, float ki, float kd);
    float compute(float target, float current);

  private:
    float _kp, _ki, _kd;
    float _integral = 0;
    float _lastError = 0;
};

#endif
