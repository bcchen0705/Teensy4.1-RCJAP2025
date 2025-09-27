#include <Arduino.h>
uint8_t buffer[8];
int index = 0;

void setup() {
  Serial.begin(115200);   // Debug
  Serial3.begin(115200);  // Connect to ESP32 Serial0 TX
}

void loop() {
  if (Serial3.available()) {
    uint8_t b = Serial3.read();

    if (index == 0 && b != 0xAA) return; // wait for start
    buffer[index++] = b;

    if (index == 8) {
      index = 0;

      if (buffer[0] == 0xAA && buffer[1] == 0xAA && buffer[7] == 0xEE) {
        uint16_t dir = buffer[2] | (buffer[3] << 8);
        uint16_t dist = buffer[4] | (buffer[5] << 8);
        uint8_t checksum = (buffer[2] + buffer[3] + buffer[4] + buffer[5]) & 0xFF;

        if (checksum == buffer[6]) {
          Serial.print("Ball Direction: ");
          Serial.print(dir);
          Serial.print("°, Distance: ");
          Serial.println(dist);
        }
      }
    }
  }
}
