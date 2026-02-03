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
  // --- 1. 進入掃描模式 (ENTER) ---
  if (!digitalRead(BTN_ENTER)) {
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

  // ------------------ 左鍵切換 Start / Data ------------------
  //if(leftState == LOW && lastLeftState == HIGH &&(millis() - lastPress) > 100){
    //showData = !showData;
    //showRun = false; // 切回 Data 時取消 Run
    //lastPress = millis();
    //if(!showData){
      //showMessage("Start");
    //}
  //}
  //lastLeftState = leftState;

  // ------------------ 右鍵在 Start 顯示 Run ------------------
  //if(rightState == LOW && lastRightState == HIGH &&(millis() - lastPress) > 100){
    //if(!showData){ // 只有 Start 畫面生效
      //showRun = !showRun;
      //lastPress = millis();
      //if(showRun){
        //showMessage("Run");
        //return false;
      //}
   // }
    //else{
     // showMessage("Start");
    //}
  //}
  ///lastRightState = rightState;

  // ------------------ Sensor ------------------
  //readBNO085Yaw();
  //readBallCam();
  //readussensor();
  //if(showData &&(millis() - lastUpdate > 200)){ 
    //display.clearDisplay();
    //showSensors(gyroData.heading, ballData.angle, lineData.valid);
    //if(rightState == LOW){
      //showMessage("scanning...");
      //Serial7.write(0xAA);
      //while(digitalRead(BUTTON_LEFT));
      //Serial7.write(0xEE);
    //}
    // Serial.print("ball_dir="); Serial.println(ballData.dir);
    //lastUpdate = millis();
  //}
  //Serial.println("debug");
  //return true;
//}
//                    外側光感
void outlinesensor(){
  if(analogRead(back_ls) >= 500){
    backtouch = true;
  }
  else{
    backtouch = false;
  }
  //if(analogRead(left_ls>= 520)){
    //lefttouch = true;
  //}
  //else{
    //lefttouch = false;
  //}
  //if(analogRead(right_ls) >= 780){
    //righttouch = true;
  //}//
  //else{
    //righttouch = false;
  //}
}
/**/
void attack(){
  readBNO085Yaw();
  linesensor();
  readBallCam();
  outlinesensor();
  //LOW FREQUENCY
  unsigned long now = millis();
  if(now - lastUpdate >= interval){
    lastUpdate = now;
    readCameraData();
    kicker_control(0);
    control.picked_up =(gyroData.pitch > 20) ? true : false;
  }
  //Serial.print("height");Serial.println(targetData.h);
  float rotate = 0;
  if(targetData.valid){
    if(targetData.y > 90){
      if(targetData.x >= 130 && targetData.x <= 190){
        rotate=0;
      }
      else if(targetData.x < 120){
        int16_t e = (120-targetData.x);
        //le = le - e;
        rotate = 0.8 * (e / 120.0);
        //le = e;
      }
      else if(targetData.x > 200){
        int16_t e = (targetData.x - 200);
        rotate = -0.8 * (e / 120.0);
      }
    }
    control.robot_heading += rotate;
    if(control.robot_heading < 45){ 
      control.robot_heading = 45;
    }
    if(control.robot_heading > 135){ 
      control.robot_heading = 135;
    }
  }
  else{
    rotate = control.robot_heading - 90;
    if(rotate > 10){
      rotate = -0.8;

    }
    else if(rotate < -10){
      rotate = 0.8; 
    }
    else{
      rotate = 0;
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
  // A : 向量合成
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
    //Serial.print("degree=");Serial.println(lineDegree);

    if (!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;
      speed_timer = millis();
      //Serial.println("LINE DETECTED !!!");
      //Serial.print("initlineDegree =");Serial.println(init_lineDegree);
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
    //Serial.print("finalDegree =");Serial.println(finalDegree);
        
    lineVx = 40.0f *cos(finalDegree * DtoR_const);
    lineVy = 40.0f *sin(finalDegree * DtoR_const);
    //Serial.print("lineVx =");Serial.println(lineVx);

    //Serial.print("lineVy =");Serial.println(lineVy);
   

    Vector_Motion((int)lineVx, (int)lineVy);

    
    return;
  }
  if(!linedetected && !overhalf && (millis() - speed_timer > 300 )){ // no line
    count = 0;
    first_detect = false;
    init_lineDegree = -1;
    speed_timer = 0;
    Serial.println("back to field");
    }
  if(!linedetected){
    first_detect = false;
    overhalf = false;
    lineVx = 0;
    lineVy = 0;
  }
  if(ballData.valid){    //有球
  Serial.print("Angle: "); Serial.println(ballData.angle);
  Serial.print("Dist: "); Serial.println(ballData.dist);

  //轉成弧度
  float moving_degree = ballData.angle;
  float ballspeed = constrain(map(ballData.dist, 40, 100, 20, 50),20, 50);
  float offset = 0;

  ballspeed = constrain(ballspeed, 20, 50);
  
  if(ballData.dist >= 70){
    moving_degree = ballData.angle;
    offset = 0;
  }
  else if(ballData.angle <= 105 && ballData.angle >= 75){
    ballspeed = 30;
    moving_degree = 90;
    offset = 0;
  }
  else{
    if (ballData.dist < 42){
      offset = 90;
    }
    float offsetRatio = exp(-0.05 * (ballData.dist - 42));
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

  Serial.printf("moving%f",moving_degree);Serial.print("");
  Serial.print("vx");Serial.println(ballVx);
  Serial.print("vy");Serial.println(ballVy);
  for(uint8_t i = 0; i < 18; i++){
    Serial.print(lineData.state >> (i)&1);
  }
  Serial.println("");
  }
}
  /*Serial.printf("cout=%d\n",count);
  if(count > 0 || overhalf){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if(!linedetected && overhalf ){
      speed_timer++;
      Serial.println("halfgree");

      lineDegree = init_lineDegree + 180;
      if(speed_timer > 5000){
        count = 0;
        lineVx = 0;
        lineVy = 0;
        first_detect = false;
        init_lineDegree = -1;
        overhalf = false;
        speed_timer = 0;
      }
    }
    if(lineDegree < 0){
      lineDegree += 360;
    }
    if(start){                                
      showMessage("LINE");
      start = false;
    }
    // Serial.print("sumX="); Serial.print(sumX);
    // Serial.print(", sumY="); Serial.print(sumY);
    // Serial.print(", average lineDegree="); Serial.println(lineDegree);
    if(first_detect == false){
      init_lineDegree = lineDegree;
      first_detect = true;
    }
    if(init_lineDegree > 270 && lineDegree < 90){
      lineDegree += 360;
    }
    else if(lineDegree > 270 && init_lineDegree < 90){
      init_lineDegree += 360;
    }
    
    diff = fabs(fmod((lineDegree - init_lineDegree), 360));
    float finalDegree;
    float speed = 40;
    Serial.printf("overhalf=%d\n",overhalf);
    //Serial.printf("overhalf = %d", overhalf);
    //Serial.printf("lineDegre = %d", lineDegree);
    if(diff > EMERGENCY_THRESHOLD){
      if(overhalf){
        finalDegree = fmod(init_lineDegree + 180, 360);
      }
      else{
        overhalf = true;
        finalDegree = lineDegree;
      }
      overhalf = true;
      finalDegree = fmod(lineDegree, 360);;
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180, 360);
      // delay(1000);
    }

    if(overhalf && (hori_half | vert_half)){
      finalDegree =  init_lineDegree + 180;
    }
    Serial.print("lineDegree=");
    Serial.println(lineDegree);
    Serial.print("finalDegree=");
    Serial.println(finalDegree);
    Serial.print("first_detect");
    Serial.println(first_detect);
    //float speed = constrain(speed_timer, -40,40);
    //Serial.printf("usData.dist_f%d, usData.dist_b%d\n", usData.dist_f, usData.dist_b);

    lineVx = speed * cos(finalDegree * DtoR_const);
    lineVy = speed * sin(finalDegree * DtoR_const);
    float angle_diff = abs(90-control.robot_heading) * DtoR_const;
    float replfront = cos(angle_diff) * ((usData.dist_f * cos(angle_diff) - 15) / 5.0);//speed down started from 60cm
    float replback = cos(angle_diff) * ((usData.dist_b * cos(angle_diff) - 15) / 15.0);//speed down started from 60cm
    replfront = 1 - constrain(replfront, 0, 1);//block from 0-1;
    replfront *= -1;//negative force
    replback = 1 - constrain(replback, 0, 1);//block from 0-1;
    replback *= 1;//positive force
    Serial.printf("replfront%f, replback%f", replfront, replback);
    if(lineVy > 0){
        lineVy += replfront * MAX_V;
    }
    if(lineVy < 0){
        lineVy += replback * MAX_V;
    }
    
    float replleft =  cos(angle_diff) * ((usData.dist_l * cos(angle_diff) - 20) / 20.0);//speed down started from 60cm
    float replright = cos(angle_diff) * ((usData.dist_r * cos(angle_diff) - 20) / 20.0);//speed down started from 60cm
    replright = 1 - constrain(replright, 0, 1);//block from 0-1;
    replright *= -1;//negative force
    replleft = 1 - constrain(replleft, 0, 1);//block from 0-1;
    replleft *= 1;//positive force
    if(lineVx > 0){
        lineVx += replright * MAX_V;
    }
    if(lineVx < 0){
        lineVx += replleft * MAX_V;
    }
    Vector_Motion(int(lineVx), int(lineVy));
  
    //Vector_Motion(0,0);
    //Serial.print("lineVx="); Serial.print(lineVx);
    //Serial.print("lineVy="); Serial.println(lineVy);*/
       //if(analogRead(back_us) < 40 && (righttouch || lefttouch)){
      //ballVy = 30;
    //}
    
    //if(lefttouch || righttouch){
      //if(targetData.h > 25 && ballVy > 0){
        //ballVy = -15;
      //}
    //}
    //if(analogRead(right_us) < 50 && ballVx > 0){
      //ballVx *= 0.5;
    //}
    //else if(analogRead(left_us) < 50 && ballVx < 0){
     // ballVx *= 0.5;
    //}
    //if(backtouch && lefttouch && targetData.h != 65535){
    //}
    //else if(backtouch && righttouch && targetData.h != 65535){
    //}

    //else{
      //catch_timer--;
      //if(catch_timer < 0){
        //catch_timer = 0;
      //}
    //}
    //Serial.println(catch_timer);
    //        降速w
    /*if(targetData.valid){
      
      if(targetData.x >= 210 && ballVx < 0){
        Serial.println("1");
        ballVx = ballVx * 0.7;
        if(targetData.h > 30 && ballVy > 0){
          ballVy = 0.7 * ballVy;
        }
      }
      else if(targetData.x <= 110 && ballVx > 0){
        Serial.println("2");
        ballVx = ballVx * 0.7;
        if(targetData.h > 30 && ballVy > 0){
          ballVy = 0.7 * ballVy;
        }
      }球門降速
      if(ballVy > 0 && targetData.h != 65535){
        Serial.println("3");
        if(targetData.h > 25){
          ballVy = 20;
        }
        else if(targetData.h < 25){
          ballVy *= 0.8;
        }
      }
    }
    else{
      if((analogRead(left_us) < 80 && ballVx < 0) || analogRead(right_us) < 80 && ballVx > 0){
        ballVx *= 0.7;
        Serial.print("0.7");
      }
      if(analogRead(left_us) < 60 && ballVx < 0 || analogRead(right_us) < 60 && ballVx > 0){
        ballVx *= 0.4;
        if(analogRead(front_us) < 40){
          ballVy = -15;
        }
        if(analogRead(front_us) < 30){
          ballVy = -30;
        }
        Serial.print("0.5");
      }
    }
    if(analogRead(left_us) < 25 && ballVx < 0 ){
      ballVx = 30;
      Serial.print("-15");
    }
    if(analogRead(right_us) < 25 && ballVx > 0){
      ballVx = -30;
    }

    // Optional debug output
    Serial.print("Vx: ");
    Serial.print(ballVx);
    Serial.print("  Vy: ");
    Serial.println(ballVy);

    Serial.printf("usData.dist_f%d, usData.dist_b%d\n", usData.dist_f, usData.dist_b);
    float angle_diff = abs(90-control.robot_heading) * DtoR_const;
    float replfront = cos(angle_diff) * ((usData.dist_f * cos(angle_diff) - 30) / 30.0);//speed down started from 60cm
    float replback = cos(angle_diff) * ((usData.dist_b * cos(angle_diff) - 25) / 25.0);//speed down started from 60cm
    replfront = 1 - constrain(replfront, 0, 1);//block from 0-1;
    replfront *= -1;//negative force
    replback = 1 - constrain(replback, 0, 1);//block from 0-1;
    replback *= 1;//positive force
    Serial.printf("usData.dist_l%d, usData.dist_r%d\n", usData.dist_l, usData.dist_r);
    float replleft =  cos(angle_diff) * ((usData.dist_l * cos(angle_diff) - 25) / 25.0);//speed down started from 60cm
    float replright = cos(angle_diff) * ((usData.dist_r * cos(angle_diff) - 25) / 25.0);//speed down started from 60cm
    replright = 1 - constrain(replright, 0, 1);//block from 0-1;
    replright *= -1;//negative force
    replleft = 1 - constrain(replleft, 0, 1);//block from 0-1;
    replleft *= 1;//positive force
    Serial.printf("replfront%f, replback%f", replfront, replback);
    if(ballVy> 0.5*MAX_V //&& (abs(replleft)>0.5 || abs(replright)>0.5)){
        ballVy += replfront * MAX_V*0.5;
    }
    if(ballVy < -0.5*MAX_V){
        ballVy += replback * MAX_V*0.5;
    }
    if(ballVx > 0.5 * MAX_V){
        ballVx += replright * MAX_V * 0.5;
    }
    if(ballVx < -0.5 * MAX_V){
        ballVx += replleft * MAX_V * 0.5;
    }
    Serial.printf("ballVx = %f, ballVy = %f\n", ballVx, ballVy);
    //Vector_Motion(0,0);
    Vector_Motion(int(ballVx), int(ballVy));
  }
  //Vector_Motion(0,0);
  Serial.println("----------");
  Serial.print("ballVy");Serial.println(ballVy);
  Serial.print("ballVx");Serial.println(ballVx);*/
  // float finalVx =(lineVx != 0) ? lineVx : ballVx + lineVx;
  // float finalVy =(lineVy != 0) ? lineVy : ballVy + lineVy;
  


  //Serial.print("Target X =");Serial.println(targetData.x);
  //Serial.print("Target Y =");Serial.println(targetData.y);
  //Serial.print("Target H =");Serial.println(targetData.h);
  //Serial.print("control.robot_heading =");Serial.println(control.robot_heading);
  //Serial.print("rotate =");Serial.println(rotate);
  //Serial.print("ballData.possession =");Serial.println(ballData.possession);
  //Serial.print("usback =");Serial.println(analogRead(back_us));
  //Serial.print("gyroData.pitch=");Serial.println(gyroData.pitch);
  //Serial.print("usleft =");Serial.println(analogRead(left_us));  
  //Serial.print("usright =");Serial.println(analogRead(right_us));
  //Serial.print("usfront =");Serial.println(analogRead(front_us));

