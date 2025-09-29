#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
//Buttons
#define BUTTON_LEFT  31
#define BUTTON_RIGHT 30
//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Motors

//Sensors
struct GyroData {float heading = 0.0; bool valid = false;} gyroData;
struct LineData {uint32_t state = 999; bool valid = false;} lineData;
struct BallData {uint8_t dir = 255; uint8_t dis = 255; bool valid = false;} ballData;

/*Sensors Part*/
/*
void readBNO085Yaw(HardwareSerial &serial) {
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];


  while (serial.available() >= PACKET_SIZE) {
    buffer[0] = serial.read();
    if (buffer[0] != 0xAA) continue;
    buffer[1] = serial.read();
    if (buffer[1] != 0xAA) continue;


    for (int i = 2; i < PACKET_SIZE; i++) buffer[i] = serial.read();


    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    gyroData.valid = true;
    gyroData.heading = yaw_raw * 0.01f; // 轉成度
    break;
  }
}*/

void readBNO085Yaw() {
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  while (Serial2.available() >= PACKET_SIZE) {
    buffer[0] = Serial2.read();
    if (buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if (buffer[1] != 0xAA) continue;
    for (int i = 2; i < PACKET_SIZE; i++) buffer[i] = Serial2.read();
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    gyroData.valid = true;
    gyroData.heading = yaw_raw * 0.01f; // 轉成度
    break;
  }
}

void ballsensor(){
  uint8_t buffer_index = 0;
  uint8_t buffer[3];
  ballData.valid = false;
  while (Serial3.available()) {
    uint8_t b = Serial3.read();
    if (buffer_index == 0 && b != 0xAA) return; // wait for start
    buffer[buffer_index++] = b;

    if (buffer_index == 3) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[2] == 0xEE) {
        uint8_t temp = buffer[1];
        ballData.valid = true;
        ballData.dir = (temp & 0x0F);
        ballData.dis = (temp & 0xF0) >> 4;
      }
    }
  }
}

void linesensor(){
  uint8_t buffer_index = 0;
  uint8_t buffer[7];
  lineData.valid = false;
  while (Serial4.available()) {
    uint8_t b = Serial4.read();
    if (buffer_index == 0 && b != 0xAA) return; // wait for start
    buffer[buffer_index++] = b;
    if (buffer_index == 7) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[0] == 0xAA && buffer[6] == 0xEE) {
        lineData.state = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16);
        uint8_t checksum = (buffer[2] + buffer[3] + buffer[4]) & 0xFF;
        if (checksum == buffer[5]) {
            lineData.valid = true;
            Serial.print("sensors: ");
            Serial.println(lineData.state, BIN);
        }
      }
    }
  }
}


/*Actuators Part*/

   
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


void SetMotorSpeed(){
  ;
}

void MotorStop(){
  ;  
}

bool MotorTest(){
  
  return true;
}


void RobotFKControl(){
  ;
}

void RobotIKControl(){
  ;
}

