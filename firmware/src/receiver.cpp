#include "receiver.h"

struct_message receivedData;

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    Serial.printf("Received -> Roll: %d | Pitch: %d | Throttle: %d | Yaw: %d\n",
                  receivedData.roll, receivedData.pitch,
                  receivedData.throttle, receivedData.yaw);
}

void receiverInit() {
    Serial.begin(115200);

    initMotors(); // initialize PWM pins

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("ESP-NOW Receiver Ready");
}
