#ifndef IMU_H
#define IMU_H
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
