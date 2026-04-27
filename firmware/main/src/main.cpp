#include "receiver.h"
#include "imu.h"
#include "motors.h"
#include "flight/flight_types.h"
#include "flight/stabilizer.h"
#include "system/flight_system.h"
#include "power/pm_manager.h"

// Declare external variables
extern unsigned long lastTime;
extern struct_message receivedData;
extern unsigned long receiverLastPacketMs;

unsigned long loopLastTime = 0;
const unsigned long LOOP_TIME_US = 4000; // 4ms = 250Hz
Stabilizer stabilizer;
FlightSystem flightSystem;
PowerManager powerManager;

void handleSerialCommands();
Setpoint decodeSetpoint();

void setup() {
    Serial.begin(115200);
    Serial.println("Starting advanced drone stack...");
    
    receiverInit();
    initIMU();
    initMotors();
    stabilizer.init(LOOP_TIME_US / 1000000.0f);
    flightSystem.init();
    powerManager.init();
    
    // Start with motors at minimum (stopped)
    setMotorSpeeds(1000, 1000, 1000, 1000);
    
    Serial.println("Ready. A=arm D=disarm S=emergency stop");
    loopLastTime = micros();
    lastTime = micros();
}

void loop() {
    static unsigned long lastRxSeenMs = 0;
    unsigned long currentTime = micros();
    
    // Run at fixed frequency (250Hz)
    if (currentTime - loopLastTime >= LOOP_TIME_US) {
        float dt = (currentTime - loopLastTime) / 1000000.0;
        loopLastTime = currentTime;
        if (dt <= 0.0f) {
            dt = LOOP_TIME_US / 1000000.0f;
        }
        
        IMUData imu = readIMU();
        Setpoint setpoint = decodeSetpoint();
        if (receiverLastPacketMs != lastRxSeenMs) {
            lastRxSeenMs = receiverLastPacketMs;
            flightSystem.onReceiverPacket();
        }
        flightSystem.update(millis(), setpoint.throttleUs);
        
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 100) {
            FlightState state = stabilizer.state();
            Serial.printf("R:%.1f P:%.1f Y:%.1f Armed:%d Rx:%d\n",
                         state.attitudeDeg.roll, state.attitudeDeg.pitch,
                         state.attitudeDeg.yaw, flightSystem.isArmed(),
                         flightSystem.receiverHealthy());
            lastPrint = millis();
        }

        MotorMix mix = stabilizer.update(setpoint, imu, dt);

        if (!flightSystem.shouldCutPower() && setpoint.throttleUs > 1100) {
            setMotorSpeeds(mix.m1, mix.m2, mix.m3, mix.m4);
        } else {
            setMotorSpeeds(1000, 1000, 1000, 1000);
        }

        powerManager.update();
    }
    
    handleSerialCommands();
}

Setpoint decodeSetpoint() {
    auto mapf = [](float x, float inMin, float inMax, float outMin, float outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    };
    Setpoint sp;
    sp.attitudeDeg.roll = mapf(receivedData.roll, 0.0f, 255.0f, -30.0f, 30.0f);
    sp.attitudeDeg.pitch = mapf(receivedData.pitch, 0.0f, 255.0f, -30.0f, 30.0f);
    sp.attitudeDeg.yaw = 0.0f;
    sp.attitudeRateDps.roll = 0.0f;
    sp.attitudeRateDps.pitch = 0.0f;
    sp.attitudeRateDps.yaw = mapf(receivedData.yaw, 0.0f, 255.0f, -180.0f, 180.0f);
    sp.throttleUs = map(receivedData.throttle, 0, 255, 1000, 2000);
    return sp;
}

void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch(cmd) {
            case 'A':
            case 'a':
                flightSystem.arm();
                Serial.println("ARMED!");
                break;
                
            case 'D':
            case 'd':
                flightSystem.disarm();
                setMotorSpeeds(1000, 1000, 1000, 1000);
                Serial.println("DISARMED");
                break;
                
            case 'S':
            case 's':
                flightSystem.emergencyStop();
                setMotorSpeeds(1000, 1000, 1000, 1000);
                Serial.println("EMERGENCY STOP!");
                break;
        }
    }
}