#include <esp_now.h>
#include <WiFi.h>

#define ADC_PIN 5  // Pin ADC (GPIO 5)

const uint32_t intervalMicros = 100;  // 66µs~15kHz 100µs-10kHz 1000µs-1kHz
uint32_t lastMicros = 0;
uint32_t lastMicrosUdp = 0;
uint32_t counter = 0;
uint32_t delta_time_us = 0;

uint32_t rawValueADC = 0;

uint8_t receiverAddress[] = { 0x94, 0xB9, 0x7E, 0x8B, 0xD3, 0x48 };  // Adresse MAC du récepteur atommatrix 24:A1:60:46:07:90   atomlite : 94:B9:7E:8B:D3:48
struct DataPacket {
  uint8_t data[3];
  //uint8_t time[3];
} packet;

void sendCallback(const uint8_t *macAddr, esp_now_send_status_t status) {
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent" : "Failed");
}

void setup() {
  Serial.begin(2000000);

  delay(1000);
  Serial.println();
  Serial.println("Start SENDER");

  analogReadResolution(12);  // Résolution ADC à 12 bits (valeurs entre 0 et 4095)

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
  rawValueADC = 0;
}

void loop() {

  if ((micros() - lastMicros) >= intervalMicros) {

    delta_time_us = (micros() - lastMicros);
    lastMicros = micros();

    // read the value from the sensor:
    rawValueADC = analogRead(ADC_PIN);
    //Serial.println(rawValueADC);

    // Remplir les données
    packet.data[0] = (rawValueADC >> 16) & 0xFF;
    packet.data[1] = (rawValueADC >> 8) & 0xFF;
    packet.data[2] = rawValueADC & 0xFF;
    // packet.time[0] = (delta_time_us >> 16) & 0xFF;
    // packet.time[1] = (delta_time_us >> 8) & 0xFF;
    // packet.time[2] = delta_time_us & 0xFF;

    //rawValueADC = (rawValueADC + 1) & 0xFF;  // Incrément modulo 24 bits 0xFFFFFF - 16bits 0xFFFF - 8bits 0xFF

    // Envoyer le paquet
    esp_now_send(receiverAddress, (uint8_t *)&packet, sizeof(packet));
  }
  //delay(1); // ~15kHz
}
