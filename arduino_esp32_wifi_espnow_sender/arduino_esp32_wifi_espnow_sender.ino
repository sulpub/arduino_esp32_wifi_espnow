#include <esp_now.h>
#include <WiFi.h>

const uint32_t intervalMicros = 66;  // 66 µs pour ~15 kHz
uint32_t lastMicros = 0;
uint32_t lastMicrosUdp = 0;
uint32_t counter = 0;
uint32_t delta_time_us = 0;

uint8_t receiverAddress[] = { 0x24, 0xA1, 0x60, 0x46, 0x07, 0x90 };  // Adresse MAC du récepteur atommatrix 24:A1:60:46:07:90
struct DataPacket {
  uint8_t data[3];
  uint8_t time[3];
} packet;

void sendCallback(const uint8_t *macAddr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent" : "Failed");
}

void setup() {
  Serial.begin(2000000);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed!");
    return;
  }
  esp_now_register_send_cb(sendCallback);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  counter = 0;
}

void loop() {

  if ((micros() - lastMicros) >= intervalMicros) {

    delta_time_us = (micros() - lastMicros);
    lastMicros = micros();

    // Remplir les données
    packet.data[0] = (delta_time_us >> 16) & 0xFF;
    packet.data[1] = (delta_time_us >> 8) & 0xFF;
    packet.data[2] = delta_time_us & 0xFF;
    packet.time[0] = (counter >> 16) & 0xFF;
    packet.time[1] = (counter >> 8) & 0xFF;
    packet.time[2] = counter & 0xFF;

    counter = (counter + 1) & 0xFFFFFF;  // Incrément modulo 24 bits

    // Envoyer le paquet
    esp_now_send(receiverAddress, (uint8_t *)&packet, sizeof(packet));
  }
  //delay(1); // ~15kHz
}
