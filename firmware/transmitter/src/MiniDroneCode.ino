#include <esp_now.h>
#include <WiFi.h>

// ✅ Define the structure to send
typedef struct struct_message {
  int throttle;
  int yaw;
  int pitch;
  int roll;
} struct_message;

struct_message dataToSend;

// ✅ Receiver MAC address (replace with your actual receiver MAC)
uint8_t receiverMAC[] = { 0x08, 0xA6, 0xF7, 0xBC, 0xFA, 0x9C }; 

// ✅ Callback after sending
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Success" : "❌ Fail");
}

void setup() {
  Serial.begin(115200);

  // ✅ Setup joystick pins as INPUT
  pinMode(12, INPUT);  // Yaw
  pinMode(14, INPUT);  // Throttle
  pinMode(2, INPUT);   // Pitch
  pinMode(15, INPUT);  // Roll

  // ✅ Set device as Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ✅ Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }
  Serial.println("✅ ESP-NOW Initialized");

  // ✅ Register send callback
  esp_now_register_send_cb(onDataSent);

  // ✅ Register peer (receiver)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  }

  Serial.println("✅ Peer Added");
}

void loop() {
  // ✅ Read joystick analog values
  dataToSend.yaw      = analogRead(12);
  dataToSend.throttle = analogRead(14);
  dataToSend.pitch    = analogRead(2);
  dataToSend.roll     = analogRead(15);

  // ✅ Print data for debugging
  Serial.printf("Yaw: %d | Throttle: %d | Pitch: %d | Roll: %d\n",
                dataToSend.yaw, dataToSend.throttle,
                dataToSend.pitch, dataToSend.roll);

  // ✅ Send data to receiver via ESP-NOW
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
  
  if (result != ESP_OK) {
    Serial.println("❌ Error sending the data");
  }

  delay(100); // Send every 100ms
}
