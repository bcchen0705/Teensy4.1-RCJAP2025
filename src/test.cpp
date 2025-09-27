#include <Robot.h>
void setup(){
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial4.begin(115200);
    Serial5.begin(115200);
    Serial6.begin(115200);
    Serial7.begin(115200);
    Serial8.begin(115200);
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);
    Wire.begin();
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    showStart();
}

int lastLeftState  = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;

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