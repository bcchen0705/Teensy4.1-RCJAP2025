#include <Robot.h>
int lightSensor = 999;
int ballSensor  = 999;
void setup(){
    Robot_Init();
    showStart();
}

int lastLeftState  = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;
bool showData = false;  // false = Start/Run, true = 顯示數據
bool showRun  = false;


void loop(){
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


  // ------------------ Sensor ------------------
  readBNO085Yaw();
  ballsensor();

  // ------------------ 顯示 Data 畫面 ------------------
  if (showData && (millis() - lastUpdate > 200)) {
    showSensors(gyroData.heading, ballData.dir, ballSensor);
    Serial.print("ball_dir="); Serial.println(ballData.dir);
    lastUpdate = millis();
  }
}


void robot_attack(){
  ;
}