#include <stdio.h>
#include <iostream>
#include <cstdint> // For uint8_t

// Define the struct for native simulation
typedef struct struct_message {
    uint8_t roll;
    uint8_t pitch;
    uint8_t throttle;
    uint8_t yaw;
} struct_message;

// Mock receiver data
struct_message receivedData = {127, 127, 50, 127}; // Center positions, low throttle

// Mock map function
float map(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

// Mock IMU data structure
struct IMUData {
    float roll;
    float pitch;
    float yaw;
};

// Mock PID class for simulation
class PID {
public:
    PID(float kp, float ki, float kd, float minOutput, float maxOutput) 
        : _kp(kp), _ki(ki), _kd(kd), _minOutput(minOutput), _maxOutput(maxOutput) {}
    
    float compute(float target, float current, float dt) {
        float error = target - current;
        // Simple P controller for simulation
        return _kp * error;
    }
    
private:
    float _kp, _ki, _kd;
    float _minOutput, _maxOutput;
};

int main() {
    printf("=== Drone Simulation Started ===\n\n");
    
    // Simulate multiple iterations
    for (int iteration = 0; iteration < 5; iteration++) {
        printf("--- Iteration %d ---\n", iteration + 1);
        
        // Simulate controller input changes
        receivedData.roll = 127 + (iteration * 10);
        receivedData.pitch = 127 - (iteration * 5);
        receivedData.throttle = 50 + (iteration * 5);
        receivedData.yaw = 127;
        
        printf("Controller Input: R=%d, P=%d, T=%d, Y=%d\n",
               receivedData.roll, receivedData.pitch, 
               receivedData.throttle, receivedData.yaw);
        
        // Convert to angle range
        float targetRoll = map(receivedData.roll, 0, 255, -30.0, 30.0);
        float targetPitch = map(receivedData.pitch, 0, 255, -30.0, 30.0);
        float targetYaw = map(receivedData.yaw, 0, 255, -180.0, 180.0);
        
        printf("Target angles: R=%.1f°, P=%.1f°, Y=%.1f°\n", targetRoll, targetPitch, targetYaw);
        
        // Simulate IMU readings (mock data with some noise)
        IMUData imu;
        imu.roll = targetRoll * 0.8f + (rand() % 10 - 5) * 0.1f;
        imu.pitch = targetPitch * 0.8f + (rand() % 10 - 5) * 0.1f;
        imu.yaw = targetYaw * 0.1f;
        
        printf("IMU readings: R=%.1f°, P=%.1f°, Y=%.1f°\n", imu.roll, imu.pitch, imu.yaw);
        
        // Compute PID outputs
        PID pidRoll(1.0, 0.05, 0.1, -100, 100);
        PID pidPitch(1.0, 0.05, 0.1, -100, 100);
        PID pidYaw(1.0, 0.01, 0.05, -100, 100);
        
        float rollOutput = pidRoll.compute(targetRoll, imu.roll, 0.02);
        float pitchOutput = pidPitch.compute(targetPitch, imu.pitch, 0.02);
        float yawOutput = pidYaw.compute(targetYaw, imu.yaw, 0.02);
        
        printf("PID outputs: R=%.1f, P=%.1f, Y=%.1f\n", rollOutput, pitchOutput, yawOutput);
        
        // Motor mixing (X configuration)
        int baseThrottle = receivedData.throttle;
        int m1 = baseThrottle + rollOutput - pitchOutput + yawOutput;
        int m2 = baseThrottle - rollOutput - pitchOutput - yawOutput;
        int m3 = baseThrottle - rollOutput + pitchOutput + yawOutput;
        int m4 = baseThrottle + rollOutput + pitchOutput - yawOutput;
        
        // Constrain motor values (0-255)
        m1 = (m1 < 0) ? 0 : (m1 > 255) ? 255 : m1;
        m2 = (m2 < 0) ? 0 : (m2 > 255) ? 255 : m2;
        m3 = (m3 < 0) ? 0 : (m3 > 255) ? 255 : m3;
        m4 = (m4 < 0) ? 0 : (m4 > 255) ? 255 : m4;
        
        printf("Motor outputs: M1=%d, M2=%d, M3=%d, M4=%d\n", m1, m2, m3, m4);
        printf("\n");
    }
    
    printf("=== Simulation Complete ===\n");
    return 0;
}