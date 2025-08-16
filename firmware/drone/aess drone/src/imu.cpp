#include "imu.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

void initIMU() {
    if (!mpu.begin()) {
        Serial.println("IMU not found!");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

IMUData readIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    IMUData data;
    data.roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    data.pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180 / PI;
    data.yaw = 0; 

    return data;
}
