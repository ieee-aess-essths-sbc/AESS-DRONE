#ifndef IMU_H
#define IMU_H
#include <Adafruit_MPU6050.h>
extern unsigned long lastTime;

struct IMUData {
    float roll;
    float pitch;
    float yaw;
};
void calibrateIMU();
void initIMU();
IMUData readIMU();

#endif
