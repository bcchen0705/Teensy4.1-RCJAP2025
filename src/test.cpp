
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h> // Added for sin, cos, and fabs
#include <stdbool.h> // Added for clarity



void setup() {
  Serial.print(9600);
  Serial5.begin(115200);
}

void loop() {
 if(Serial5.available()){
    Serial.print("RX: ");
    Serial.println(Serial5.read(), HEX);
  } else {
    Serial.println("no data");
    delay(500);
  }
}
