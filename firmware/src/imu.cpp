#include "imu.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
//variables for sensor fusion
float accelRoll, accelPitch, accelYaw;
float gyroRoll, gyroPitch, gyroYaw;
float compRoll, compPitch, compYaw; //check this comp thingy 
unsigned long lastTime = 0; 
float dt;

//calibration offsets-prepare a simple doc on this plz
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
void calibrateIMU(){
    Serial.println("Calibrating IMU...");
    delay(1000);
    float gx = 0, gy = 0, gz = 0;
    const int numSamples = 100; // number of readings for calibration
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gx += g.gyro.x;
        gy += g.gyro.y;
        gz += g.gyro.z;
        delay(2);
    }
    gyroXoffset = gx / numSamples;
    gyroYoffset = gy / numSamples;
    gyroZoffset = gz / numSamples;
    Serial.printf("IMU Calibration Complete\nX: %f\nY: %f\nZ: %f\n", gyroXoffset, gyroYoffset, gyroZoffset);
}

void initIMU() {
    if (!mpu.begin()) {
        Serial.println("IMU not found!");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateIMU();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelRoll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180 / PI;
    compRoll = accelRoll;
    compPitch = accelPitch;
    compYaw = 0;
    lastTime = micros();
}

IMUData readIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
        unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
    lastTime = currentTime;
    float gyroX = g.gyro.x - gyroXoffset;
    float gyroY = g.gyro.y - gyroYoffset;
    float gyroZ = g.gyro.z - gyroZoffset;
    accelRoll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180 / PI;
    gyroRoll += gyroX * dt * 180/PI;  // Converting stuff again
    gyroPitch += gyroY * dt * 180/PI;
    gyroYaw += gyroZ * dt * 180/PI;
    compRoll = 0.98 * (compRoll + gyroX * dt * 180/PI) + 0.02 * accelRoll;
    compPitch = 0.98 * (compPitch + gyroY * dt * 180/PI) + 0.02 * accelPitch;
    compYaw = gyroYaw;  // Yaw from gyro only (no magnetometer)
    IMUData data;
    data.roll = compRoll;
    data.pitch = compPitch;
    data.yaw = compYaw;
    
    return data;
}
