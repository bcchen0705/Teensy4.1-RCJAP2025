
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#define SENSE(i) (!(lineData.state >> (i)&1))

#define possession_Threshold 205

//SPEED
float ballVx = 0;
float ballVy = 0;
float lineVx = 0;
float lineVy = 0;

float x = 2;
float init_lineDegree = -1;
float diff = 0;
bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;
uint32_t speed_timer = 0;

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

//GOAL
uint32_t lastTargetTime = 0;
static float rotate = 0;
static bool isRecovering = false; // 紀錄是否正在處理邊緣回彈

int lastLeftState = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;
bool showData = false; // false = Start/Run, true = 顯示數據
bool showRun = false;
// #define DEBUG
void robot_offense();
bool Debug();
bool ball_search();
void offense();
void line_processing();
void outlinesensor();
void attack();

void setup(){
  Robot_Init();
  showMessage("Start");
  display.clearDisplay();
}

bool Debug() {
  readBNO085Yaw();
  readBallCam();
  linesensor(); 
  readCameraData();
  // --- 顯示球的數據 (新增部分) ---
  static uint32_t lastDisplayTime = 0;
  if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    display.println("--- DEBUG MODE ---");

    // 顯示球的資訊
    display.setCursor(0, 20);
    if (ballData.valid) {
      display.printf("Ball: DETECTED\n");
      display.printf("Angle: %d\n", ballData.angle);
      display.printf("Dist:  %d\n", ballData.dist);
    } else {
      display.printf("Ball: NOT FOUND\n");
    }

    // 顯示指南針 (Heading) 輔助確認感測器是否正常
    display.setCursor(0, 45);
    display.printf("Yaw: %.1f", gyroData.heading);
    //display.printf("pitch: %.1f", gyroData.pitch);
    display.printf("x: %d", targetData.x);
    display.printf("y: %d",targetData.y );
    display.printf("w: %d", targetData.w);
    display.printf("h: %d",targetData.h );
    display.display();
    lastDisplayTime = millis();
  }
  // --- 進入掃描模式 (ENTER) ---
  if (!digitalRead(BTN_ENTER)) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println("SCANNING...");
    display.display();

    Serial.println("Enter Scanning Mode");
    Serial7.write(0xAA); 
    delay(300); // 防止按鍵抖動

    // 在掃描期間，依然要維持讀取數據，才看得到結果
    // 這裡用 while 迴圈卡住，直到「再次按下 ESC」
    while (digitalRead(BTN_ESC)) { 
      delay(20); 
    }

    Serial7.write(0xEE); // 發送結束/儲存指令
    Serial.println("Save and Exit Scanning");
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("SAVED!");
    display.display();
    
    delay(500); // 給 ESP32 時間寫入 EEPROM，並防止 ESC 鍵誤觸下一個判斷
  }

  // --- 2. 退出 Debug 模式 (按住 UP 鍵或是長按 ESC) ---
  // 改用 BTN_UP 進入 attack，避免跟掃描模式的 ESC 鍵打架
  if (!digitalRead(BTN_UP)) {
    Serial.println(">>> Exiting Debug. Starting Attack!");
    return false; // 跳出 while(Debug()) 進入 attack
  }

  return true;
}

void loop(){
  while(Debug());
  while (1) {
    attack(); // 執行你的比賽邏輯
  }
}

void attack(){
  readBNO085Yaw();
  linesensor();
  readBallCam();

// --- 關閉螢幕 ---
  static bool screenCleared = false;
  if (!screenCleared) {
    display.clearDisplay();
    display.display(); 
    screenCleared = true;
    Serial.println("OLED Blackout for Competition Mode");
  }
// -------------------------

// --- 重製 ---
  if (fabs(gyroData.pitch) > 15) {
    lineVx = 0;lineVy = 0;
    ballVx = 0;ballVy = 0;
    first_detect = false; 
    init_lineDegree = -1;
    overhalf = false;
    isRecovering = false;
    // 讓機器人放下後暫時朝向正前方，避免噴走
    control.robot_heading = 90; 
    
    Vector_Motion(0, 0); 
    Serial.println("ROBOT PICKED UP - ALL STATES RESET");
    return; 
  }
// -------------------------

  unsigned long now = millis();

  //轉向球門
  if(now - lastCameraUpdate >= interval){
    lastCameraUpdate = now;
    readCameraData();
    if(targetData.valid){
      lastTargetTime = now;

      // --- 1. 邊緣觸發：當球門太偏，進入「強制修正模式」 ---
      if (targetData.x < 90 || targetData.x > 210) {
        isRecovering = true;
      }

      if (isRecovering) {
        // 強制轉向，直到球門回到中間區間 (140~180)
        if (targetData.x < 140) rotate = 0.18;
        else if (targetData.x > 180) rotate = -0.18;
        else {
            isRecovering = false; // 回到中間了，解除強制模式
            rotate = 0;
        }
      } 
      else {
        // --- 2. 核心修正：球門在中間了，但車身如果是斜的，要慢慢回正到 90 ---
        // 這裡就是解決你說「回到中間轉不回來」的關鍵
        if (control.robot_heading > 110) {
            rotate = -0.18; // 緩慢向左修正
        } else if (control.robot_heading < 80) {
            rotate = 0.18;  // 緩慢向右修正
        } else {
            rotate = 0;
            control.robot_heading = 90; // 進入死區，強制鎖定 90
        }
      }
    }
    else{ //沒球門
      if(now - lastTargetTime < 1000){rotate =0 ;}
      else{
        if(control.robot_heading > 95) {rotate = -0.18f;}
        else if(control.robot_heading < 85){rotate = 0.18f;}
        else{rotate = 0;}
      }
    }
  }
  
  control.robot_heading += rotate;
  if(targetData.h > 30){
  control.robot_heading = constrain(control.robot_heading, 30, 150);}
  else{control.robot_heading = constrain(control.robot_heading, 45, 135);}
/*  檢查光感

static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) { 
    Serial.print("Teensy Rx Bin: ");
    for (int i = 17; i >= 0; i--) {
      Serial.print(!bitRead(lineData.state, i));
      if (i % 4 == 0 && i != 0) Serial.print(" "); // 每 8 位空一格
    }
    Serial.println();
    lastPrint = millis();
  }

*/


//-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  for(int i = 0; i < 18; i++){
    if(!bitRead(lineData.state, i)){
      //if(i==0){continue;}
      Serial.printf("read%d", i);
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
      linedetected = true;
    }
  }

  // B : 反彈

  if(linedetected && count > 1){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0){lineDegree += 360;} 
    Serial.print("degree=");Serial.println(lineDegree);

    if (!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;
      speed_timer = millis();
      Serial.println("LINE DETECTED !!!");
      Serial.print("initlineDegree =");Serial.println(init_lineDegree);
    }

    diff = fabs(lineDegree - init_lineDegree);
    if(diff > 180){diff = 360 - diff;}
    Serial.print("diff =");Serial.println(diff);


    ////-----BACK TO FIELD-----
    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = fmod(init_lineDegree + 180.0f,360.0f);
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    Serial.print("finalDegree =");Serial.println(finalDegree);
        
    lineVx = 50.0f *cos(finalDegree * DtoR_const);
    lineVy = 50.0f *sin(finalDegree * DtoR_const);
    Vector_Motion(lineVx, lineVy);
    Serial.print("lineVx =");Serial.println(lineVx);
    Serial.print("lineVy =");Serial.println(lineVy);
   
  }
  else{
    init_lineDegree = -1;
    diff = 0;
    lineVx = 0;
    lineVy = 0;
    overhalf = false;
    speed_timer = 0;
    first_detect = false;


    //-----BALL SENSOR-----

    if(ballData.valid){    //有球
      Serial.print("Angle: "); Serial.println(ballData.angle);
      Serial.print("Dist: "); Serial.println(ballData.dist);

      //轉成弧度
      float moving_degree = ballData.angle;
      float ballspeed = constrain(map(ballData.dist, 25, 35, 15, 35),20, 35);
      float offset = 0;

      ballspeed = constrain(ballspeed, 20, 35);
      
      if(ballData.dist >= 40){
        moving_degree = ballData.angle;
        offset = 0;
      }
      else if(ballData.angle <= 119 && ballData.angle >= 69){
        ballspeed = 30;
        moving_degree = 90;
        offset = 0;
        if(ballData.dist <= 18){
          ballspeed = 35;
        }
      }
      else{
        if (ballData.dist <= 25){
          offset = 90;
        }
        float offsetRatio = exp(-0.05 * (ballData.dist - 25));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 45 + 45 * offsetRatio;
        
        float side;
        if(ballData.angle > 90 && ballData.angle < 270){
          side = 1;
        }
        else{side = -1;}

        moving_degree = ballData.angle + (offset * side);
        moving_degree = fmod(moving_degree + 360.0f, 360.0f) ;

      }
        
      //計算vx vy
      ballVx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
      ballVy = (int)round(ballspeed * sin(moving_degree * DtoR_const));
      //減速
      if(targetData.valid){
        if(targetData.h >=35 && ballVy > 0){ballVy *= 0.7;}
      }
      Serial.printf("moving%f",moving_degree);Serial.print("");
      Serial.print("vx");Serial.println(ballVx);
      Serial.print("vy");Serial.println(ballVy);

      Vector_Motion(ballVx,ballVy);
    }
    else { //無球
      Serial.println("No Ball Detected");
      Vector_Motion(0,0);
    }   
  }
}
