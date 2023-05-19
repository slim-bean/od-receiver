#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <atomic>

#define ARM_RELAY_1 4
#define ARM_RELAY_2 5
#define FIRE_RELAY_1 6

// This mac address
// 78:21:84:88:6F:F4
// 10:91:A8:40:D7:24
// Transmitter mac address
// E8:9F:6D:0A:2F:70

uint8_t broadcastAddress[] = {0xE8, 0x9F, 0x6D, 0x0A, 0x2F, 0x70};

enum Status {
  SAFE,
  ARMING,
  ARMED
};

std::atomic<Status> armed(SAFE);


typedef struct status_message {
    uint8_t messageType;
    Status status;
} status_message;

typedef struct control_message {
    uint8_t messageType;
    bool arm;
    bool fire;
} control_message;

status_message statusMessage;

control_message controlMessage;

esp_now_peer_info_t peerInfo;


void armTask(void* pvParameters){ 
    Serial.println("Arming!");
    armed.store(ARMING);
    neopixelWrite(RGB_BUILTIN, 128, 128, 0);
    // Open relay 1 for 5 seconds
    digitalWrite(ARM_RELAY_1, LOW);
    delay(3000);
    digitalWrite(ARM_RELAY_1, HIGH);
    
    delay(500);
    
    // Open relay 2 for 5 seconds
    digitalWrite(ARM_RELAY_2, LOW);
    delay(3000);
    digitalWrite(ARM_RELAY_2, HIGH);
    
    neopixelWrite(RGB_BUILTIN, 128, 0, 0);
    Serial.println("Armed!");
    armed.store(ARMED);
    vTaskDelete(NULL);
}

void fireTask(void* pvParameters){ 
    Serial.println("Fire!");
    // Enable fire relay
    digitalWrite(FIRE_RELAY_1, LOW);
    delay(500);
    digitalWrite(FIRE_RELAY_1, HIGH);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    armed.store(SAFE);
    vTaskDelete(NULL);
}

// Callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                  : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    switch (incomingData[0]) {
        case 1:
            Serial.println("Status received");
            break;
        case 2:
            Serial.println("Control received");
            memcpy(&controlMessage, incomingData, sizeof(controlMessage));
            if (controlMessage.arm && armed.load() == SAFE) {
              xTaskCreate(armTask, "ArmTask", 1000, NULL, 2, NULL);
            } else if (controlMessage.fire) {
              xTaskCreate(fireTask, "FireTask", 1000, NULL, 2, NULL);
            } else {
              Serial.println("Invalid command");
            }
            break;

        default:
            Serial.println("Received unknown message type");
            break;
    }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  neopixelWrite(RGB_BUILTIN, 0, 0, 0);

  pinMode(ARM_RELAY_1, OUTPUT); 
  digitalWrite(ARM_RELAY_1, HIGH);
  pinMode(ARM_RELAY_2, OUTPUT); 
  digitalWrite(ARM_RELAY_2, HIGH);
  pinMode(FIRE_RELAY_1, OUTPUT); 
  digitalWrite(FIRE_RELAY_1, HIGH);

  // Set device as a Wi-Fi Station
  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);

  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
  }
  // Register for a callback function that will be called when data is
  // received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // Send message via ESP-NOW
    statusMessage.messageType = 1;
    statusMessage.status = armed.load();
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&statusMessage,
                                    sizeof(statusMessage));
    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }
    delay(500);
}
