#include "receiver.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"

// Initialize PIDs with reasonable values and output limits
// These will need tuning for our drone
PID pidRoll(1.0, 0.05, 0.1, -100, 100);
PID pidPitch(1.0, 0.05, 0.1, -100, 100);
PID pidYaw(1.0, 0.01, 0.05, -100, 100); // KP, KI, KD, MinOutput, MaxOutput
//timing variables
const int LOOP_TIME = 4000; //4ms = 250Hz loop rate

void setup() {
    Serial.begin(115200);
    receiverInit();  // initialize ESP-NOW receiver
    initIMU();
    initMotors();
    setMotorSpeeds(0, 0, 0, 0); // Start with motors off
    Serial.println("Drone initialization");
    delay(1000);
}

void loop() {
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
    lastTime = currentTime;
    IMUData imu = readIMU();

    // Convert received data from 0-255 to appropriate ranges
    // Assuming center at 127, convert to angle range (e.g., -30 to 30 degrees)
    float targetRoll = map(receivedData.roll, 0, 255, -30.0, 30.0);
    float targetPitch = map(receivedData.pitch, 0, 255, -30.0, 30.0);
    float targetYaw = map(receivedData.yaw, 0, 255, -180.0, 180.0);
    // Compute PID outputs
    float rollOutput = pidRoll.compute(targetRoll, imu.roll, dt);
    float pitchOutput = pidPitch.compute(targetPitch, imu.pitch, dt);
    float yawOutput = pidYaw.compute(targetYaw, imu.yaw, dt);
    //Get base throttle
    int baseThrottle = receivedData.throttle;
    // Standard quadcopter X-configuration mixing
    // Front-left (M1): throttle + roll - pitch + yaw
    // Front-right (M2): throttle - roll - pitch - yaw
    // Rear-right (M3): throttle - roll + pitch + yaw
    // Rear-left (M4): throttle + roll + pitch - yaw
    int m1 = baseThrottle + rollOutput - pitchOutput + yawOutput;
    int m2 = baseThrottle - rollOutput - pitchOutput - yawOutput;
    int m3 = baseThrottle - rollOutput + pitchOutput + yawOutput;
    int m4 = baseThrottle + rollOutput + pitchOutput - yawOutput;
    
    // Set motor speeds
    setMotorSpeeds(m1, m2, m3, m4);

}
