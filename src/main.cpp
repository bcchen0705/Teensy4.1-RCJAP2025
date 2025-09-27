#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

// ------------------ OLED ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// ------------------ 按鍵 ------------------
#define BUTTON_LEFT  31
#define BUTTON_RIGHT 30
bool showData = false;  // false = Start/Run, true = 顯示數據
bool showRun  = false;


// ------------------ 模擬數據 ------------------
int lightSensor = 999;
int ballSensor  = 999;
float gyroYaw   = 0;


// ------------------ OLED 顯示函式 ------------------
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


// ------------------ BNO085 UART ------------------
#define BNO_BAUD 115200
#define BNO_RX 7  // Teensy 4.1 RX7 -> Serial2


// ------------------ setup ------------------
void setup() {
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);


  Serial.begin(115200);
  Serial2.begin(BNO_BAUD); // BNO085 UART


  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);


  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  showStart();
}


// ------------------ loop ------------------
int lastLeftState  = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;


// 解析 BNO085 UART (簡單讀 Yaw)
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
    gyroYaw = yaw_raw * 0.01f; // 轉成度
    break;
  }
}


void loop() {
  int leftState  = digitalRead(BUTTON_LEFT);
  int rightState = digitalRead(BUTTON_RIGHT);


  // ------------------ 左鍵切換 Start / Data ------------------
  if (leftState == LOW && lastLeftState == HIGH && (millis() - lastPress) > 100) {
    showData = !showData;
    showRun = false; // 切回 Data 時取消 Run
    lastPress = millis();
    if (!showData) showStart();
  }
  lastLeftState = leftState;


  // ------------------ 右鍵在 Start 顯示 Run ------------------
  if (rightState == LOW && lastRightState == HIGH && (millis() - lastPress) > 100) {
    if (!showData) {  // 只有 Start 畫面生效
      showRun = !showRun;
      lastPress = millis();
      if (showRun) showRunScreen();
      else showStart();
    }
  }
  lastRightState = rightState;


  // ------------------ 讀取 BNO085 Gyro ------------------
  readBNO085Yaw();


  // ------------------ 顯示 Data 畫面 ------------------
  if (showData && (millis() - lastUpdate > 200)) {
    showSensors(gyroYaw, lightSensor, ballSensor);
    lastUpdate = millis();
  }
}



