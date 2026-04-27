#include <Servo.h>

const int FR_PIN = 3; 
const int FL_PIN = 5; 
const int BR_PIN = 6; 
const int BL_PIN = 9;

Servo motors[4];
int motorPins[] = {FR_PIN, FL_PIN, BR_PIN, BL_PIN};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 4; i++) {
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(1000); // Sendin minimum throttle
  }
  delay(2000); 
  Serial.println("Motor Test Ready. Send 1, 2, 3, or 4 to spin a motor.");
}

void loop() {
  if (Serial.available() > 0) {
    int motorIdx = Serial.parseInt() - 1;
    if (motorIdx >= 0 && motorIdx < 4) {
      Serial.print("Spinning Motor: "); Serial.println(motorIdx + 1);
      
      motors[motorIdx].writeMicroseconds(1200); 
      delay(2000);
      motors[motorIdx].writeMicroseconds(1000);
    }
  }
}