#include <esp_now.h>
#include <WiFi.h>

uint32_t counter = 0;
uint32_t delta_time_us = 0;

struct DataPacket {
  uint8_t data[3];
  uint8_t time[3];
};

void onDataReceive(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
  /*
    packet.data[0] = (delta_time_us >> 16) & 0xFF;
    packet.data[1] = (delta_time_us >> 8) & 0xFF;
    packet.data[2] = delta_time_us & 0xFF;
    packet.time[0] = (counter >> 16) & 0xFF;
    packet.time[1] = (counter >> 8) & 0xFF;
    packet.time[2] = counter & 0xFF;
*/

  DataPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));
  counter=((packet.data[0] << 16) + (packet.data[1] << 8) + (packet.data[2]));
  delta_time_us=((packet.time[0] << 16) + (packet.time[1] << 8) + (packet.time[2]));
  

  Serial.print(counter);
  Serial.print(",");
  Serial.print(delta_time_us);
  /*
  for (int i = 0; i < 3; i++) Serial.print(packet.data[i], HEX);
  Serial.print(",");
  for (int i = 0; i < 3; i++) Serial.print(packet.time[i], HEX);
  */
  Serial.println();
}

void setup() {
  Serial.begin(1500000);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed!");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);
}

void loop() {
  // Réception automatique via onDataReceive
}
