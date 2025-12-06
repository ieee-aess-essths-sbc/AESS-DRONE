#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "motors.h"  // include motor pin constants

// Structure for joystick data
typedef struct struct_message {
    uint8_t roll;
    uint8_t pitch;
    uint8_t throttle;
    uint8_t yaw;
} struct_message;

// Make received data accessible in main.cpp
extern struct_message receivedData;

// Function prototypes
void receiverInit();
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

#endif
