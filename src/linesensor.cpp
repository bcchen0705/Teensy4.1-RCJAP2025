#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

float lineVx = 0;
float lineVy = 0;

float init_lineDegree = -1;
float diff = 0;

bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;
uint32_t speed_timer = 0;


void setup() {
  Robot_Init();
}

void loop(){
    readBNO085Yaw();
    linesensor();

    float sumX = 0.0f, sumY = 0.0f;
    int count = 0;
    bool linedetected = false;

  for(int i = 0; i < 18; i++){
    if(bitRead(lineData.state, i)){
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
      linedetected = true;
    }
  }

    // B : 反彈

  if(linedetected && count > 0){
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

    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = fmod(init_lineDegree +180.0f,360.0f);
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    Serial.print("finalDegree =");Serial.println(finalDegree);
        
    lineVx = 40.0f *cos(finalDegree * DtoR_const);
    lineVy = 40.0f *sin(finalDegree * DtoR_const);
    
   
  }
  else{
    init_lineDegree = -1;
    diff = 0;
    lineVx = 0;
    lineVy = 0;
    overhalf = false;
    speed_timer = 0;
    first_detect = false;
  }
  Vector_Motion(lineVx, lineVy);
  Serial.print("lineVx =");Serial.println(lineVx);
  Serial.print("lineVy =");Serial.println(lineVy);
}