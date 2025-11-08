#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define possession_Threshold 205

//SPEED
float ballVx = 0;
float ballVy = 0;
float lineVx = 0;
float lineVy = 0;

int count = 0;
float x = 2;
float init_lineDegree = -1;
float diff = 0;
bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;




//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 10Hz = 每100毫秒一次

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
void attack();


void setup(){
  Robot_Init();
  showMessage("Start");
}

void loop(){
  //Debug();
  while(Debug());
  while(1){
    attack();
  }  
}

bool Debug(){
  int leftState = digitalRead(BUTTON_LEFT);
  int rightState = digitalRead(BUTTON_RIGHT);

  // ------------------ 左鍵切換 Start / Data ------------------
  if(leftState == LOW && lastLeftState == HIGH &&(millis() - lastPress) > 100){
    showData = !showData;
    showRun = false; // 切回 Data 時取消 Run
    lastPress = millis();
    if(!showData){
      showMessage("Start");
    }
  }
  lastLeftState = leftState;

  // ------------------ 右鍵在 Start 顯示 Run ------------------
  if(rightState == LOW && lastRightState == HIGH &&(millis() - lastPress) > 100){
    if(!showData){ // 只有 Start 畫面生效
      showRun = !showRun;
      lastPress = millis();
      if(showRun){
        showMessage("Run");
        return false;
      }
    }
    else{
      showMessage("Start");
    }
  }
  lastRightState = rightState;

  // ------------------ Sensor ------------------
  readBNO085Yaw();
  ballsensor();
  //readussensor();
  if(showData &&(millis() - lastUpdate > 200)){ 
    display.clearDisplay();
    showSensors(gyroData.heading, ballData.dir, lineData.valid);
    showUS(usData.dist_l, usData.dist_r, usData.dist_b);
    if(rightState == LOW){
      showMessage("scanning...");
      Serial5.write(0xAA);
      while(digitalRead(BUTTON_LEFT));
      Serial5.write(0xEE);
    }
    // Serial.print("ball_dir="); Serial.println(ballData.dir);
    lastUpdate = millis();
  }
  Serial.println("debug");
  return true;
}

void attack(){
  static int catch_timer = 0;
  static int16_t le = 0;
  static int16_t re = 0;
  //HIGH FREQUENCY
  readBNO085Yaw();
  linesensor();
  readussensor();
  //LOW FREQUENCY
  unsigned long now = millis();
  if(now - lastUpdate >= interval){
    lastUpdate = now;
    readCameraData();
    kicker_control(0);
    control.picked_up =(gyroData.pitch > 20) ? true : false;
  }
  //Serial.print("height");Serial.println(targetData.h);
  double rotate = 0;
  if(targetData.valid){
    if(targetData.y > 90){
      Serial.println(targetData.x);
      if(targetData.x >= 130 && targetData.x <= 190){
        rotate=0;
        le = 0;
        re = 0;
      }
      else if(targetData.x < 130){
        int16_t e = (130-targetData.x);
        le = le - e;
        rotate = e / 130.0 + le * 0.1;
        le = e;
      }
      else if(targetData.x > 190){
        int16_t e = (targetData.x - 190);
        re = re - e;
        rotate = -e / 130.0 - re * 0.1;
        re = e;
      }
    }
    control.robot_heading += rotate;
    if(control.robot_heading < 40){ 
      control.robot_heading = 40;
    }
    if(control.robot_heading > 140){ 
      control.robot_heading = 140;
    }
  }
  else{
    rotate = control.robot_heading - 90;
    if(rotate > 10){
      rotate = -1;

    }
    else if(rotate < -10){
      rotate = 1; 
    }
    control.robot_heading += rotate; 
  }
  
  /*Serial.print(" Y=");
  Serial.print(targetData.y);
  Serial.print(" W=");
  Serial.print(targetData.w);
  Serial.print(" H=");
  Serial.println(targetData.h);*/
  
  // Serial.print("lineData.state=");
  // Serial.println(lineData.state, BIN);
  //Serial.print("dist_f = ");Serial.println(dist_f);
  //Serial.print("dist_b = ");Serial.println(dist_b );
  //Serial.print("dist_l = ");Serial.println(dist_l);
  //Serial.print("dist_r = ");Serial.println(dist_r);
  float sumX = 0, sumY = 0;
  for(int i = 0; i < 18; i++){
    bool detected =((lineData.state &(1UL << i)) == 0); // 0 表有線
    // Serial.print("Sensor "); Serial.print(i);
    // Serial.print(": "); Serial.println(detected ? "ON line" : "OFF line");
    if(detected){
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
    }
  }
  count = 0;
  if(lineData.state == 0b111111111111111111 && !overhalf && count > 0){ // no line
    count = 0;
    lineVx = 0;
    lineVy = 0;
    first_detect = false;
    init_lineDegree = -1;
  }
  if(control.picked_up){
    count = 0;
    lineVx = 0;
    lineVy = 0;
    first_detect = !first_detect;
    init_lineDegree = -1;
    overhalf = !overhalf;
  }
  if(count > 0 || overhalf){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if(lineDegree < 0){
      lineDegree += 360;
    }
    if(start){
      showMessage("Line");
      start = false;
    }
    // Serial.print("sumX="); Serial.print(sumX);
    // Serial.print(", sumY="); Serial.print(sumY);
    // Serial.print(", average lineDegree="); Serial.println(lineDegree);
    if(first_detect == false){
      init_lineDegree = lineDegree;
      first_detect = true;
    }

    diff = fabs(fmod((lineDegree - init_lineDegree), 360));
    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = lineDegree;
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180, 360);
      // delay(1000);
    }
    /*
    Serial.print("lineDegree=");
    Serial.println(lineDegree);
    Serial.print("finalDegree=");
    Serial.println(finalDegree);
    Serial.print("first_detect");
    Serial.println(first_detect);
    */
    float speed = 50;
    lineVx = speed * cos(finalDegree * DtoR_const);
    lineVy = speed * sin(finalDegree * DtoR_const);
    Vector_Motion(int(lineVx), int(lineVy));
    //Serial.print("lineVx="); Serial.print(lineVx);
    //Serial.print("lineVy="); Serial.println(lineVy);
  }
  else{
    lineVx = 0;
    lineVy = 0;
    if(!start){
      showMessage("Start");
      start = true;
    }
    ballsensor();
    if(ballData.dir == 255){ 
      Vector_Motion(0,0);
      /*if(usData.dist_b <= 110 && usData.dist_b >= 90){
        ballVy = 0;
      }
      else if(usData.dist_b < 90 ){
        ballVy = 25;
      }
      else if(usData.dist_b > 110){
        ballVy = -25;
      }
    Serial.print("ballVy");Serial.println(ballVy);
    

    if((abs(usData.dist_r - usData.dist_l)) <= 30){
        ballVx = 0;
      }
      else if(abs(usData.dist_r - usData.dist_l) > 30 && usData.dist_r > usData.dist_l){
        ballVx = 25;
      }
      else if(abs(usData.dist_r - usData.dist_l) > 30 && usData.dist_r < usData.dist_l){
        ballVx = -25;
      }*/
    //Serial.print("ballVx");Serial.println(ballVx);
    }
    else{
      float ballDegree = ballDegreelist[ballData.dir];
      float offset = 0;

      //Serial.print("balldir=");Serial.println(ballData.dir);
      //Serial.print("ballDegree=");Serial.println(ballDegree);
      //Serial.print("ballData.dis=");Serial.println(ballData.dis);
      // Serial.print("exp");Serial.println(exp(-0.55*(ballData.dis-7)));
      //delay(500);
      float ballspeed = map(ballData.dis, 0, 12, 20, 50);
      if(ballDegree == 87.5 || ballDegree == 92.5){
        offset = 0;
        ballspeed = MAX_V * 0.7;
      }
      else{
        double offsetRatio = exp(-0.55 *(ballData.dis - 7));
        offsetRatio =(exp(-0.55 *(ballData.dis - 7)) > 1) ? 1 : offsetRatio;
        offset = 95 * offsetRatio;
        offset =(ballDegree > 90) ? offset : -offset;
        offset =(ballDegree < 270) ? offset : -offset;
        //Serial.print("offset=");
        //Serial.println(offset);
      }
      float moving_Degree = ballDegree + offset;
      // Serial.print("moving_Degree="); Serial.println(moving_Degree);
      ballspeed = constrain(ballspeed, 20, 60);
      // Serial.print("BallSpeed="); Serial.println(ballspeed);
      ballVx = ballspeed * cos(moving_Degree * DtoR_const);
      ballVy = ballspeed * sin(moving_Degree * DtoR_const);
    }
    if(backtouch && ballVy < 0){
      ballVy = 0;
    }
    if(lefttouch && ballVx < 0){
      ballVx = 0;
    }
    if(righttouch && ballVx > 0){
      ballVx = 0;
    }

    /*if(targetData.h > 40){
      if(ballVy > 0){
        ballVy = ballVy * 0.5;
      }
    }*/

    if(ballData.possession < possession_Threshold){
      ballVy = MAX_V;
      catch_timer++;
      if(catch_timer > 5){
        if(targetData.valid){
          if(targetData.y > 126){
            kicker_control(1);
            Serial.print("kick");
            catch_timer = 0;
          }
        }
      }
    }
    else{
      catch_timer--;
      if(catch_timer < 0){
        catch_timer = 0;
      }
    }
    //Serial.println(catch_timer);
    //Vector_Motion(int(ballVx), int(ballVy));
    Vector_Motion(0,0);
  }
  // float finalVx =(lineVx != 0) ? lineVx : ballVx + lineVx;
  // float finalVy =(lineVy != 0) ? lineVy : ballVy + lineVy;
  
  if(digitalRead(back_ls) == 0){
    backtouch = false;
  }
  if(digitalRead(left_ls) == 0){
    lefttouch = false;
  }
  if(digitalRead(right_ls) == 0){
    righttouch = false;
  }
  //Serial.print("Target X =");Serial.println(targetData.x);
  Serial.print("Target Y =");Serial.println(targetData.y);
  //Serial.print("control.robot_heading =");Serial.println(control.robot_heading);
  //Serial.print("rotate =");Serial.println(rotate);
}