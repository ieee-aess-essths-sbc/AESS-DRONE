#include <Arduino.h>
#include "receiver.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"

PID pidRoll(1.0, 0.0, 0.0);   // example
PID pidPitch(1.0, 0.0, 0.0);
PID pidYaw(1.0, 0.0, 0.0);

void setup() {
    Serial.begin(115200);
    initReceiver();  // initialize ESP-NOW receiver
    initIMU();
    initMotors();
}

void loop() {
    IMUData imu = readIMU();

    float rollOutput  = pidRoll.compute(incomingData.roll, imu.roll);
    float pitchOutput = pidPitch.compute(incomingData.pitch, imu.pitch);
    float yawOutput   = pidYaw.compute(incomingData.yaw, imu.yaw);

    setMotorSpeeds(
        incomingData.throttle + rollOutput - pitchOutput + yawOutput,
        incomingData.throttle - rollOutput - pitchOutput - yawOutput,
        incomingData.throttle - rollOutput + pitchOutput + yawOutput,
        incomingData.throttle + rollOutput + pitchOutput - yawOutput
    );

    delay(20);
}
