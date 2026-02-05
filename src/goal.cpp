#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#define SENSE(i) (!(lineData.state >> (i)&1))

#define possession_Threshold 205



//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

//GOAL
uint32_t lastTargetTime = 0;
bool wasTargetValid = false;

void setup(){
  Robot_Init();
  showMessage("Start");
  display.clearDisplay();
}


static float rotate = 0;
void loop(){
  readBNO085Yaw();
  readCameraData();
  unsigned long now = millis();
  if(now - lastCameraUpdate >= interval){
    lastCameraUpdate = now;
    if(targetData.valid){
      static bool isRecovering = false; // 紀錄是否正在處理邊緣回彈
      lastTargetTime = now;

      // --- 1. 邊緣觸發：當球門太偏，進入「強制修正模式」 ---
      if (targetData.x < 90 || targetData.x > 210) {
        isRecovering = true;
      }

      if (isRecovering) {
        // 強制轉向，直到球門回到中間區間 (140~180)
        if (targetData.x < 140) rotate = 0.8;
        else if (targetData.x > 180) rotate = -0.8;
        else {
            isRecovering = false; // 回到中間了，解除強制模式
            rotate = 0;
        }
      } 
      else {
        // --- 2. 核心修正：球門在中間了，但車身如果是斜的，要慢慢回正到 90 ---
        // 這裡就是解決你說「回到中間轉不回來」的關鍵
        if (control.robot_heading > 92) {
            rotate = -0.3; // 緩慢向左修正
        } else if (control.robot_heading < 88) {
            rotate = 0.3;  // 緩慢向右修正
        } else {
            rotate = 0;
            control.robot_heading = 90; // 進入死區，強制鎖定 90
        }
      }
      /*
      if(targetData.x >= 100 && targetData.x <= 220){
        if(control.robot_heading > 100) {rotate = -0.8f;}
        else if(control.robot_heading < 80){rotate = 0.8f;}
        else{rotate = 0;}
      }
      else if(targetData.x < 100){
        int16_t e = (100.0f - targetData.x);
        rotate = 0.8f * (e / 100.0f);
      }
      else if(targetData.x > 220){
        int16_t e = (targetData.x - 220.0f);
        rotate = -0.8f * (e / 100.0f);
      }*/
    }
    else{ //沒球門
      if(now - lastTargetTime < 3000){rotate =0 ;}
      else{
        if(control.robot_heading > 95) {rotate = -0.8f;}
        else if(control.robot_heading < 85){rotate = 0.8f;}
        else{rotate = 0;}
      }
    }

  
  control.robot_heading += rotate;
  if(targetData.h > 30){
  control.robot_heading = constrain(control.robot_heading, 50, 140);}
  else{control.robot_heading = constrain(control.robot_heading, 40, 140);}

  Vector_Motion(0,0);

  Serial.print(" X=");Serial.print(targetData.x);
  Serial.print(" Y=");Serial.print(targetData.y);
  Serial.print(" W=");Serial.print(targetData.w);
  Serial.print(" H=");Serial.println(targetData.h);
  }
}