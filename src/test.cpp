
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h> // Added for sin, cos, and fabs
#include <stdbool.h> // Added for clarity


int LDR_pin = A0;
int LDR_value = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  LDR_value = analogRead(LDR_pin);  // 0~1023 (或 0~4095 看 Teensy 型號)
  Serial.println(LDR_value);
  delay(100);
}
