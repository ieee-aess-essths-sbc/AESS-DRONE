#include "receiver.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"

// Declare external variables
extern unsigned long lastTime;
extern struct_message receivedData;

// Initialize PIDs
PID pidRoll(1.0, 0.05, 0.1, -100, 100);
PID pidPitch(1.0, 0.05, 0.1, -100, 100);
PID pidYaw(1.0, 0.01, 0.05, -100, 100);

// Timing and safety
unsigned long loopLastTime = 0;
const unsigned long LOOP_TIME_US = 4000; // 4ms = 250Hz
bool armed = false;

// ADD THIS FUNCTION DECLARATION
void handleSerialCommands();

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Drone Initialization...");
    
    receiverInit();
    initIMU();
    initMotors();
    
    // Start with motors at minimum (stopped)
    setMotorSpeeds(1000, 1000, 1000, 1000);
    
    Serial.println("Drone Ready. Send 'A' to arm, 'D' to disarm.");
    loopLastTime = micros();
    lastTime = micros();  // Initialize IMU timer too
}

void loop() {
    unsigned long currentTime = micros();
    
    // Run at fixed frequency (250Hz)
    if (currentTime - loopLastTime >= LOOP_TIME_US) {
        float dt = (currentTime - loopLastTime) / 1000000.0;
        loopLastTime = currentTime;
        
        // Read IMU
        IMUData imu = readIMU();
        
        // DEBUG: Print angles
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 100) {
            Serial.printf("Roll: %.1f | Pitch: %.1f | Yaw: %.1f\n", 
                         imu.roll, imu.pitch, imu.yaw);
            lastPrint = millis();
        }
        
        // Convert receiver data
        float targetRoll = map(receivedData.roll, 0, 255, -30.0, 30.0);
        float targetPitch = map(receivedData.pitch, 0, 255, -30.0, 30.0);
        float targetYaw = map(receivedData.yaw, 0, 255, -180.0, 180.0);
        int throttle = map(receivedData.throttle, 0, 255, 1000, 2000);
        
        // Compute PID outputs
        float rollOutput = pidRoll.compute(targetRoll, imu.roll, dt);
        float pitchOutput = pidPitch.compute(targetPitch, imu.pitch, dt);
        float yawOutput = pidYaw.compute(targetYaw, imu.yaw, dt);
        
        // Motor mixing (X-configuration)
        // Motor order: M1(1)=FL, M2(5)=FR, M3(2)=BL, M4(13)=BR
        int m1 = throttle - rollOutput + pitchOutput + yawOutput;  // FL
        int m2 = throttle + rollOutput + pitchOutput - yawOutput;  // FR
        int m3 = throttle - rollOutput - pitchOutput - yawOutput;  // BL
        int m4 = throttle + rollOutput - pitchOutput + yawOutput;  // BR
        
        // Apply only if armed
        if (armed && throttle > 1100) {
            setMotorSpeeds(m1, m2, m3, m4);
        } else {
            // Motors stopped
            setMotorSpeeds(1000, 1000, 1000, 1000);
        }
    }
    
    // Handle serial commands
    handleSerialCommands();
}

// ADD THIS FUNCTION DEFINITION
void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch(cmd) {
            case 'A':
            case 'a':
                armed = true;
                Serial.println("ARMED!");
                break;
                
            case 'D':
            case 'd':
                armed = false;
                setMotorSpeeds(1000, 1000, 1000, 1000);
                Serial.println("DISARMED");
                break;
                
            case 'S':
            case 's':
                armed = false;
                setMotorSpeeds(1000, 1000, 1000, 1000);
                Serial.println("EMERGENCY STOP!");
                break;
        }
    }
}