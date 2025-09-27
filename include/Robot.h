#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#define BUTTON_LEFT  31
#define BUTTON_RIGHT 30
// ------------------ OLED ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);




struct GyroData {float heading = 0.0; bool valid = false;} gyroData;

void showStart() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println("Start");
  display.display();
}


void showRunScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println("Run");
  display.display();
}


void showSensors(float gyro, int light, int ball) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("Gyro: ");  display.println(gyro);
  display.setCursor(0, 15); display.print("Light: "); display.println(light);
  display.setCursor(0, 30); display.print("Ball: ");  display.println(ball);
  display.display();
}



void readBNO085Yaw(HardwareSerial &seerial) {
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];


  while (Serial.available() >= PACKET_SIZE) {
    buffer[0] = Serial.read();
    if (buffer[0] != 0xAA) continue;
    buffer[1] = Serial.read();
    if (buffer[1] != 0xAA) continue;


    for (int i = 2; i < PACKET_SIZE; i++) buffer[i] = Serial.read();


    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    gyroData.valid = true;
    gyroData.heading = yaw_raw * 0.01f; // 轉成度
    break;
  }
}