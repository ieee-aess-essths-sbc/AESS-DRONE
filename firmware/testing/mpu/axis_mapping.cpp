#include <Wire.h>
#include <math.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68

int16_t AccX, AccY, AccZ;

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);

  // Waking up our MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  // Readin accelerometer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  AccX = Wire.read() << 8 | Wire.read();
  AccY = Wire.read() << 8 | Wire.read();
  AccZ = Wire.read() << 8 | Wire.read();

  // Convertion
  float pitch = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;
  float roll  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Roll: ");
  Serial.println(roll);

  delay(200);
}