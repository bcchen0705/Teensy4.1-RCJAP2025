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
  
  {
    /* code */
  }
  
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
  /*if (rightState == LOW && lastRightState == HIGH && (millis() - lastPress) > 100) {
    if (!showData) {  // 只有 Start 畫面生效
      showRun = !showRun;
      lastPress = millis();
      if (showRun) showRunScreen();
      else showStart();
    }
  }
  lastRightState = rightState;*/


  // ------------------ Sensor ------------------
  readBNO085Yaw();
  ballsensor();

  // ------------------ 顯示 Data 畫面 ------------------
  if (showData && (millis() - lastUpdate > 200)) {
    showSensors(gyroData.heading, ballData.dir, ballSensor);
    Serial.print("ball_dir="); Serial.println(ballData.dir);
    lastUpdate = millis();
  }
if (ballData.dir==3 || ballData.dir==4){
  RobotIKControl (0,15,0);
  }
else if (ballData.dir==0||ballData.dir==2){
  RobotIKControl (-15,0,0);
}
else if (ballData.dir==5||ballData.dir==7){
  RobotIKControl (15,0,0); 
}
else if (ballData.dir==8|| ballData.dir==10||ballData.dir==13||ballData.dir==15){
  RobotIKControl (0,-15,0);
}
else{ 
  RobotIKControl (0,0,0);
}
/*switch (ballData.dir) {
  case 3:
  case 4:
    RobotIKControl(0, 15, 0);
    break;

  case 0:
  case 2:
    RobotIKControl(15, 0, 0);
    break;

  case 5:
  case 7:
    RobotIKControl(-15, 0, 0);
    break;

  case 8:
  case 10:
  case 13:
  case 15:
    RobotIKControl(0, -15, 0);
    break;

  default:
    RobotIKControl(0, 0, 0);
    break;
}
*/




}


void robot_attack(){
  ;
}
