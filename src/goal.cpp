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
  readCameraData();
  
  unsigned long now = millis();
  static float lastVisualError = 0;
  static float finalHeading = 90.0f;
  uint32_t currentTime = millis();

  //轉向球門
  if(now - lastCameraUpdate >= interval){
    lastCameraUpdate = now;
    


    if(targetData.valid){
      lastTargetTime = now;
      
      float VisualError = (160.0f - float(targetData.x)) / 160.0f;

      finalHeading = 90.0f + (VisualError * 55.0f);

    }
    else{
      if(now - lastTargetTime > 2000){
        finalHeading = 90.0f;
      }
    }
  }
      /*if(targetData.x >= 100 && targetData.x <= 220){
        if(control.robot_heading > 110) {rotate = -0.1f;}
        else if(control.robot_heading < 70){rotate = 0.1f;}
        else{rotate = 0;}
      }
      else if(targetData.x < 100){
        int16_t e = (100.0f - targetData.x);
        rotate = 0.2f * (e / 100.0f);
      }
      else if(targetData.x > 220){
        int16_t e = (targetData.x - 220.0f);
        rotate = -0.2f * (e / 100.0f);
      }
    }
    else{
      if(now - lastTargetTime < 3000){rotate =0 ;}
      else{
        if(control.robot_heading > 90) {rotate = -0.1f;}
        else if(control.robot_heading < 90){rotate = 0.1f;}
        else{rotate = 0;}
      }
    }
  }
  */
  control.robot_heading = finalHeading;
  control.robot_heading = constrain(control.robot_heading, 45, 135);

  Vector_Motion(0,0);
  Serial.print("heading=");Serial.println(control.robot_heading);
  //Serial.print("rotate=");Serial.println(rotate);

  Serial.print(" X=");Serial.print(targetData.x);
  Serial.print(" Y=");Serial.print(targetData.y);
  Serial.print(" W=");Serial.print(targetData.w);
  Serial.print(" H=");Serial.println(targetData.h);
}