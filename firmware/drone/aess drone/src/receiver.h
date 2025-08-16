#ifndef RECEIVER_H
#define RECEIVER_H

#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
    int throttle;
    int yaw;
    int pitch;
    int roll;
} struct_message;

// Declare only, defined in receiver.cpp
extern struct_message incomingData;

// Functions
void initReceiver();
void onReceiveData(const uint8_t *mac_addr, const uint8_t *data, int len);

#endif
