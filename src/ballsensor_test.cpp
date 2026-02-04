#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>


float ballVx;
float ballVy;
float lineVx;
float lineVy;

float x = 2;
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
    readBallCam();
    linesensor();

//-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  for(int i = 0; i < 18; i++){
    if(!bitRead(lineData.state, i)){
      Serial.printf("read%d", i);
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
        
    lineVx = 40.0f *cos(finalDegree * DtoR_const);
    lineVy = 40.0f *sin(finalDegree * DtoR_const);
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
      float ballspeed = constrain(map(ballData.dist, 30, 70, 20, 50),20, 50);
      float offset = 0;

      ballspeed = constrain(ballspeed, 20, 50);
      
      if(ballData.dist >= 50){
        ballspeed = 40;
        moving_degree = ballData.angle;
        offset = 0;
      }
      else if(ballData.angle <= 105 && ballData.angle >= 75){
        ballspeed = 40;
        moving_degree = 90;
        offset = 0;
      }
      else{
        if (ballData.dist <= 30){
          offset = 90;
        }
        float offsetRatio = exp(-0.05 * (ballData.dist - 30));
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
      ballVx = round(ballspeed * cos(moving_degree * DtoR_const));
      ballVy = round(ballspeed * sin(moving_degree * DtoR_const));

      Serial.printf("moving%f",moving_degree);Serial.print("");
      Serial.print("vx");Serial.println(ballVx);
      Serial.print("vy");Serial.println(ballVy);

      //誤差
      //float error = ballData.angle - gyroData.heading;

      //if(fabs(error) > 20.0f){
        //gyroData.control.robot_heading = ballData.angle;
      //}
      Vector_Motion(ballVx,ballVy);
    }
    else { //無球
      Serial.println("No Ball Detected");
      Vector_Motion(0,0);
    }
  }
    
}