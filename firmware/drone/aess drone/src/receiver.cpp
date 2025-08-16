#include "receiver.h"

struct_message incomingData; // define only here

// Callback
void onReceiveData(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len == sizeof(incomingData)) {
        memcpy(&incomingData, data, sizeof(incomingData));
        Serial.printf("Throttle:%d Yaw:%d Pitch:%d Roll:%d\n", 
                      incomingData.throttle, incomingData.yaw,
                      incomingData.pitch, incomingData.roll);
    }
}

// Initialize ESP-NOW receiver (called from main setup)
void initReceiver() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onReceiveData);
}
