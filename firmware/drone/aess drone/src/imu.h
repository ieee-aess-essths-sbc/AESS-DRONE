#ifndef IMU_H
#define IMU_H

struct IMUData {
    float roll;
    float pitch;
    float yaw;
};

void initIMU();
IMUData readIMU();

#endif
