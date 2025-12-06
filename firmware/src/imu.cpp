#include "imu.h"
#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#define MPU_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

float accelRoll, accelPitch;
float gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;
float compRoll, compPitch, compYaw;
unsigned long lastTime = 0;
float dt;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;
float accelScale = 16384.0; 
float gyroScale = 131.0;     

void calibrateIMU() {
    Serial.println("Calibrating IMU...");
    delay(2000);
    
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;
    const int numSamples = 200;
    
    for (int i = 0; i < numSamples; i++) {
        // Read raw data
        int16_t rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ;
        
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);
        
        rawAccX = Wire.read() << 8 | Wire.read();
        rawAccY = Wire.read() << 8 | Wire.read();
        rawAccZ = Wire.read() << 8 | Wire.read();
        Wire.read() << 8 | Wire.read();
        rawGyroX = Wire.read() << 8 | Wire.read();
        rawGyroY = Wire.read() << 8 | Wire.read();
        rawGyroZ = Wire.read() << 8 | Wire.read();
        
        // Accumulate
        ax += rawAccX;
        ay += rawAccY;
        az += rawAccZ;
        gx += rawGyroX;
        gy += rawGyroY;
        gz += rawGyroZ;
        
        delay(5);
    }
    
    // Calculate offsets
    accelXoffset = ax / numSamples;
    accelYoffset = ay / numSamples;
    accelZoffset = (az / numSamples) - accelScale;  // Subtract 1g
    
    gyroXoffset = gx / numSamples;
    gyroYoffset = gy / numSamples;
    gyroZoffset = gz / numSamples;
    
    Serial.printf("Calibration Complete:\n");
    Serial.printf("Accel Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", 
                  accelXoffset, accelYoffset, accelZoffset);
    Serial.printf("Gyro Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", 
                  gyroXoffset, gyroYoffset, gyroZoffset);
}

void initIMU() {
    Wire.begin(3, 4);  
    Wire.setClock(400000);
    delay(100);
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00); 
    Wire.endTransmission();
    
    // Configure accelerometer 
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x10); 
    Wire.endTransmission();
    accelScale = 4096.0;
    
    // Configure gyroscope 
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x08);  
    Wire.endTransmission();
    gyroScale = 65.5;
    
    // Calibrate
    calibrateIMU();
    
    // Initial angle calculation
    int16_t rawAccX, rawAccY, rawAccZ;
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    
    rawAccX = Wire.read() << 8 | Wire.read();
    rawAccY = Wire.read() << 8 | Wire.read();
    rawAccZ = Wire.read() << 8 | Wire.read();
    
    float ax = (rawAccX - accelXoffset) / accelScale;
    float ay = (rawAccY - accelYoffset) / accelScale;
    float az = (rawAccZ - accelZoffset) / accelScale;
    
    // Initial angles from accelerometer
    accelRoll = atan2(ay, sqrt(ax*ax + az*az)) * 180 / PI;
    accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;
    compRoll = accelRoll;
    compPitch = accelPitch;
    compYaw = 0;
    lastTime = micros();
    Serial.println("IMU initialized successfully!");
}

IMUData readIMU() {
    int16_t rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ;
    
    // Read all 14 bytes at once (accel + temp + gyro)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    
    rawAccX = Wire.read() << 8 | Wire.read();
    rawAccY = Wire.read() << 8 | Wire.read();
    rawAccZ = Wire.read() << 8 | Wire.read();
    Wire.read() << 8 | Wire.read();  
    rawGyroX = Wire.read() << 8 | Wire.read();
    rawGyroY = Wire.read() << 8 | Wire.read();
    rawGyroZ = Wire.read() << 8 | Wire.read();
    
    // Calculate time delta
    unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;
    
    // Convert to physical units
    float ax = (rawAccX - accelXoffset) / accelScale;
    float ay = (rawAccY - accelYoffset) / accelScale;
    float az = (rawAccZ - accelZoffset) / accelScale;
    
    float gx = (rawGyroX - gyroXoffset) / gyroScale;
    float gy = (rawGyroY - gyroYoffset) / gyroScale;
    float gz = (rawGyroZ - gyroZoffset) / gyroScale;
    
    // Calculate angles from accelerometer
    accelRoll = atan2(ay, sqrt(ax*ax + az*az)) * 180 / PI;
    accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;
    
    // Integrate gyroscope
    gyroRoll += gx * dt;
    gyroPitch += gy * dt;
    gyroYaw += gz * dt;
    
    // Complementary filter (96% gyro, 4% accel)
    float alpha = 0.96;
    compRoll = alpha * (compRoll + gx * dt) + (1 - alpha) * accelRoll;
    compPitch = alpha * (compPitch + gy * dt) + (1 - alpha) * accelPitch;
    compYaw = gyroYaw;
    
    // Keep angles between -180 and 180
    if (compRoll > 180) compRoll -= 360;
    if (compRoll < -180) compRoll += 360;
    if (compPitch > 180) compPitch -= 360;
    if (compPitch < -180) compPitch += 360;
    if (compYaw > 180) compYaw -= 360;
    if (compYaw < -180) compYaw += 360;
    
    IMUData data;
    data.roll = compRoll;
    data.pitch = compPitch;
    data.yaw = compYaw;
    
    return data;
}