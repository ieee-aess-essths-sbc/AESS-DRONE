#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Define your custom I2C pins
#define I2C_SDA 3
#define I2C_SCL 4

Adafruit_MPU6050 mpu;

float roll = 0, pitch = 0;
unsigned long lastTime;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial Monitor to open

    // CRITICAL: Initialize I2C with your specific pins
    Wire.begin(I2C_SDA, I2C_SCL);

    Serial.println("Starting MPU6050 verification on SDA:3, SCL:4...");

    if (!mpu.begin()) {
        Serial.println("Error: MPU6050 not found! Check wiring and pull-up resistors.");
        while (1) delay(10);
    }

    // Configure sensor for drone dynamics
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    lastTime = micros();
}

void loop() {
    // 1. Precise Loop Timing
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    // 2. Get Data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 3. Calculate Tilt (Accelerometer)
    // We use atan2 to convert raw acceleration into degrees
    float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    float accelAngleY = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;

    // 4. Complementary Filter (Fusion)
    // 96% Gyro (Fast/Smooth) + 4% Accel (Stable/No Drift)
    roll = 0.96 * (roll + g.gyro.x * dt * 180 / PI) + 0.04 * accelAngleX;
    pitch = 0.96 * (pitch + g.gyro.y * dt * 180 / PI) + 0.04 * accelAngleY;

    // 5. Validation Output for PlatformIO Serial Plotter
    // To see the graph in VSCode, use the 'Serial Plotter' extension
    Serial.print(">Roll_Angle:");
    Serial.println(roll);
    Serial.print(">Pitch_Angle:");
    Serial.println(pitch);
    Serial.print(">Loop_Hz:");
    Serial.println(1.0 / dt);

    // Maintain 200Hz refresh rate (5000us)
    while (micros() - currentTime < 5000);
}