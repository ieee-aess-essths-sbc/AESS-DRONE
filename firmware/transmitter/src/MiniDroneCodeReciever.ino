#include <esp_now.h>
#include <WiFi.h>

// Your struct (same as transmitter)
typedef struct struct_message {
  int throttle;
  int yaw;
  int pitch;
  int roll;
} struct_message;

struct_message incomingData;

// ✅ Correct callback signature for ESP32 Core v3.x
void onReceiveData(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, data, sizeof(incomingData));

    Serial.println("📡 Data Received:");
    Serial.print("Throttle: "); Serial.println(incomingData.throttle);
    Serial.print("Yaw     : "); Serial.println(incomingData.yaw);
    Serial.print("Pitch   : "); Serial.println(incomingData.pitch);
    Serial.print("Roll    : "); Serial.println(incomingData.roll);
    Serial.println("--------");
  } else {
    Serial.print("❌ Wrong data length: ");
    Serial.println(len);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  // ✅ Register the correct callback
  esp_now_register_recv_cb(onReceiveData);

  Serial.println("✅ Receiver ready!");
}

void loop() {
  // No need to do anything here
}
