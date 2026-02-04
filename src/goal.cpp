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

void loop(){
  readBNO085Yaw();

  unsigned long now = millis();
  if(now - lastCameraUpdate >= interval){
    lastCameraUpdate = now;
    readCameraData();
  
  }

  

  
  uint32_t currentTime = millis();
  float rotate = 0;

  if(targetData.valid){
    lastTargetTime = currentTime;

      if(targetData.x >= 100 && targetData.x <= 220){
        if(control.robot_heading > 100) {
        rotate = -0.2f;
        }
        else if(control.robot_heading < 80){
          rotate = 0.2f;
        }
        else{
          rotate = 0;
        }
      }
      else if(targetData.x < 100){
        int16_t e = (100-targetData.x);
        rotate = 0.2f * (e / 100.0f);

      }
      else if(targetData.x > 220){
        int16_t e = (targetData.x - 220);
        rotate = -0.2f * (e / 100.0f);
      }
    }
  else{
    if(currentTime - lastTargetTime > 3000){
      if(control.robot_heading > 100) {
        rotate = -0.2;
      }
      else if(control.robot_heading < 80){
        rotate = 0.2f;
      }
      else{
        rotate = 0;
      }
    }
  }

  control.robot_heading += rotate;
  if(control.robot_heading < 45){ control.robot_heading = 45;}
  if(control.robot_heading > 135){ control.robot_heading = 135;}

  
  //Serial.print(" heading");Serial.println(control.robot_heading);
  

  Serial.print(" X=");Serial.print(targetData.x);
  Serial.print(" Y=");Serial.print(targetData.y);
  Serial.print(" W=");Serial.print(targetData.w);
  Serial.print(" H=");Serial.println(targetData.h);
  //delay(300);
}