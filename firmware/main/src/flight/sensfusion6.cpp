#include "sensfusion6.h"
#include <Arduino.h>
#include <math.h>

void SensFusion6::init(float rollDeg, float pitchDeg, float yawDeg) {
    _att.roll = rollDeg;
    _att.pitch = pitchDeg;
    _att.yaw = yawDeg;
}

void SensFusion6::update(float gxDps, float gyDps, float gzDps,
                         float axG, float ayG, float azG, float dt) {
    
    // Filter coefficient: 0.98 is usually better for vibrations than 0.96
    const float alpha = 0.98f; 
    
    // Calculate Roll and Pitch from Accelerometer
    // We add 0.001f to avoid division by zero during high-G maneuvers
    float accelRoll = atan2f(ayG, sqrtf(axG * axG + azG * azG) + 0.001f) * 180.0f / PI;
    float accelPitch = atan2f(-axG, sqrtf(ayG * ayG + azG * azG) + 0.001f) * 180.0f / PI;

    // Complementary Filter Logic:
    // (New Angle) = Alpha * (Angle + Gyro Integration) + (1 - Alpha) * (Accel Angle)
    _att.roll = alpha * (_att.roll + gxDps * dt) + (1.0f - alpha) * accelRoll;
    _att.pitch = alpha * (_att.pitch + gyDps * dt) + (1.0f - alpha) * accelPitch;
    
    // Yaw is integrated solely from Gyro (unless you add a Magnetometer)
    _att.yaw += gzDps * dt;

    // Wrap Yaw to keep it within -180 to +180 degrees
    if (_att.yaw > 180.0f)  _att.yaw -= 360.0f;
    if (_att.yaw < -180.0f) _att.yaw += 360.0f;
}

AttitudeDeg SensFusion6::attitude() const {
    return _att;
}
