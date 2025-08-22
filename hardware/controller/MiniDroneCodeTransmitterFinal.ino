#include <WiFi.h>
#include <esp_now.h>

// Replace with your receiver's MAC address
uint8_t receiverAddress[] = {0xF4, 0x65, 0x0B, 0x56, 0x15, 0xA4};

// Structure to hold joystick data
typedef struct struct_message {
  uint8_t roll;      // use 0–255 range (mapped)
  uint8_t yaw;
  uint8_t throttle;
  uint8_t pitch;
} struct_message;

struct_message dataToSend;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  // Joystick pins (adjust to your wiring)
  pinMode(35, INPUT);
  pinMode(34, INPUT);
  pinMode(33, INPUT);
  pinMode(32, INPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW Transmitter Ready");
}

void loop() {
  // Read analogs (0–4095) and map to 0–255
  dataToSend.roll     = map(analogRead(33), 0, 4095, 0, 255);
  dataToSend.pitch    = map(analogRead(34), 0, 4095, 0, 255);
  dataToSend.throttle = map(analogRead(32), 0, 4095, 0, 255);
  dataToSend.yaw      = map(analogRead(35), 0, 4095, 0, 255);

  Serial.printf("Sending -> Roll: %d | Pitch: %d | Throttle: %d | Yaw: %d\n",
                dataToSend.roll, dataToSend.pitch, dataToSend.throttle, dataToSend.yaw);

  // Send the struct
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

  if(result != ESP_OK){
    Serial.println("Error sending data");
  }

  delay(20); // send every 20ms (~50Hz update rate)
}
