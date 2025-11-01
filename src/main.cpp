#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define RtoD_const 57.2958

#define US_front A13
#define US_left A12
#define US_right A14
#define US_back A8

float dist_f;
float dist_b;
float dist_l;
float dist_r;

const int Pin = 41;
const int Pin1 = 40;
const int Pin2 = 39;

volatile bool backtouch = false;
volatile bool lefttouch = false;
volatile bool righttouch = false;

float ballVx = 0;
float ballVy = 0;
float lineVx = 0;
float lineVy = 0;

int count = 0;
float x = 2;
const float EMERGENCY_THRESHOLD = 80.0;
float init_lineDegree = -1;
float diff = 0;
bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;

void back();
void left();
void right();

int lastLeftState = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;
bool showData = false; // false = Start/Run, true = 顯示數據
bool showRun = false;
// #define DEBUG
robotState state = IDLE;
void robot_offense();
void Debug();
bool ball_search();
void offense();
void line_processing();
void attack();

void setup()
{
  pinMode(Pin, INPUT_PULLUP);
  pinMode(Pin1, INPUT_PULLUP);
  pinMode(Pin2, INPUT_PULLUP);

  pinMode(US_front, INPUT_PULLUP);
  pinMode(US_back, INPUT_PULLUP);
  pinMode(US_left, INPUT_PULLUP);
  pinMode(US_right, INPUT_PULLUP);

  Robot_Init();
  showStart();

  attachInterrupt(digitalPinToInterrupt(Pin), back, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin1), left, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin2), right, RISING);

  Serial.begin(9600);
}

void loop()
{
  /*readBNO085Yaw();
  ballsensor();
  robot_offense();*/
  Debug();
}

void Debug()
{
  int leftState = digitalRead(BUTTON_LEFT);
  int rightState = digitalRead(BUTTON_RIGHT);

  // ------------------ 左鍵切換 Start / Data ------------------
  if (leftState == LOW && lastLeftState == HIGH && (millis() - lastPress) > 100)
  {
    showData = !showData;
    showRun = false; // 切回 Data 時取消 Run
    lastPress = millis();
    if (!showData)
      showStart();
  }
  lastLeftState = leftState;

  // ------------------ 右鍵在 Start 顯示 Run ------------------
  if (rightState == LOW && lastRightState == HIGH && (millis() - lastPress) > 100)
  {
    if (!showData)
    { // 只有 Start 畫面生效
      showRun = !showRun;
      lastPress = millis();
      if (showRun)
      {
        showRunScreen();
        while (1)
        {
          attack();
        }
      }
    }
    else
    {
      showStart();
    }
  }
  lastRightState = rightState;

  // ------------------ Sensor ------------------
  readBNO085Yaw();
  ballsensor();
  if (showData && (millis() - lastUpdate > 200))
  {
    showSensors(gyroData.heading, ballData.dir, lineData.valid);
    if (rightState == LOW)
    {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 20);
      display.println("Scanning...");
      display.display();
      Serial5.write(0xAA);
      while (digitalRead(BUTTON_LEFT))
        ;
      Serial5.write(0xEE);
    }
    // Serial.print("ball_dir="); Serial.println(ballData.dir);
    lastUpdate = millis();
  }
}

void attack(){
  readBNO085Yaw();
  linesensor();
  float sumX = 0, sumY = 0;
  // Serial.print("lineData.state=");
  // Serial.println(lineData.state, BIN);
  dist_f = analogRead(US_front) * 520 / 1024;
  dist_b = analogRead(US_back) * 520 / 1024;
  dist_l = analogRead(US_left) * 520 / 1024;
  dist_r = analogRead(US_right) * 520 / 1024;
  //Serial.print("dist_f = ");Serial.println(dist_f);
  //Serial.print("dist_b = ");Serial.println(dist_b );
  //Serial.print("dist_l = ");Serial.println(dist_l);
  //Serial.print("dist_r = ");Serial.println(dist_r);
  for (int i = 0; i < 18; i++)
  {
    bool detected = ((lineData.state & (1UL << i)) == 0); // 0 表有線
    // Serial.print("Sensor "); Serial.print(i);
    // Serial.print(": "); Serial.println(detected ? "ON line" : "OFF line");
    if (detected)
    {
      float deg = linesensorDegreelist[i];
      if (deg >= 360)
        deg -= 360;

      // Serial.print("  deg +180 = "); Serial.println(deg);

      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
    }
  }
  if (lineData.state == 0b111111111111111111 && !overhalf && count > 0)
  { // no line
    count = 0;
    lineVx = 0;
    lineVy = 0;
    first_detect = false;
    init_lineDegree = -1;
  }

  if (count > 0 || overhalf)
  {
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0)
    {
      lineDegree += 360;
    }
    if (start)
    {
      showLine();
      start = false;
    }
    // Serial.print("sumX="); Serial.print(sumX);
    // Serial.print(", sumY="); Serial.print(sumY);
    // Serial.print(", average lineDegree="); Serial.println(lineDegree);
    if (first_detect == false)
    {
      init_lineDegree = lineDegree;
      first_detect = true;
    }

    diff = fabs(fmod((lineDegree - init_lineDegree), 360));
    float finalDegree;
    if (diff > EMERGENCY_THRESHOLD)
    {
      overhalf = true;
      finalDegree = lineDegree;
      Serial.println("EMERGENCY");
    }
    else
    {
      overhalf = false;
      finalDegree = fmod(lineDegree + 180, 360);
      // delay(1000);
    }
    /*Serial.print("lineDegree=");
    Serial.println(lineDegree);
    Serial.print("finalDegree=");
    Serial.println(finalDegree);
    Serial.print("first_detect");
    Serial.println(first_detect);*/
    float speed = 50;
    lineVx = speed * cos(finalDegree * DtoR_const);
    lineVy = speed * sin(finalDegree * DtoR_const);
    Vector_Motion(int(lineVx), int(lineVy));
    // Serial.print("lineVx="); Serial.print(lineVx);
    // Serial.print("lineVy="); Serial.println(lineVy);
  }
  else
  {
    lineVx = 0;
    lineVy = 0;
    if (!start)
    {
      showStart();
      start = true;
    }
    ballsensor();
    if (ballData.dir == 255)
    {
      if((abs(dist_f - dist_b)) <= 30){
        ballVy = 0;
      }
      else if(abs(dist_f - dist_b) > 30 && dist_f > dist_b){
        ballVy = 15;
      }
      else if(abs(dist_f - dist_b) > 30 && dist_f < dist_b){
        ballVy = -15;
      }
    //Serial.print("ballVy");Serial.println(ballVy);
    

    if((abs(dist_r - dist_l)) <= 30){
        ballVx = 0;
      }
      else if(abs(dist_r - dist_l) > 30 && dist_r > dist_l){
        ballVx = 15;
      }
      else if(abs(dist_r - dist_l) > 30 && dist_r < dist_l){
        ballVx = -15;
      }
    //Serial.print("ballVx");Serial.println(ballVx);
    }
    else
    {
      float ballDegree = ballDegreelist[ballData.dir];
      float offset = 0;

      //Serial.print("balldir=");Serial.println(ballData.dir);
      //Serial.print("ballDegree=");Serial.println(ballDegree);
      //Serial.print("ballData.dis=");Serial.println(ballData.dis);
      // Serial.print("exp");Serial.println(exp(-0.55*(ballData.dis-7)));
      //delay(500);

      if (ballDegree == 87.5 || ballDegree == 92.5)
      {
        offset = 0;
      }

      else
      {

        double offsetRatio = exp(-0.55 * (ballData.dis - 7));
        offsetRatio = (exp(-0.55 * (ballData.dis - 7)) > 1) ? 1 : offsetRatio;
        offset = 95 * offsetRatio;
        offset = (ballDegree > 90) ? offset : -offset;
        offset = (ballDegree < 270) ? offset : -offset;
        //Serial.print("offset=");
        //Serial.println(offset);
      }

      float moving_Degree = ballDegree + offset;
      // Serial.print("moving_Degree="); Serial.println(moving_Degree);

      float ballspeed = map(ballData.dis, 0, 12, 20, 50);
      ballspeed = constrain(ballspeed, 20, 50);
      // Serial.print("BallSpeed="); Serial.println(ballspeed);
      ballVx = ballspeed * cos(moving_Degree * DtoR_const);
      ballVy = ballspeed * sin(moving_Degree * DtoR_const);
    }
    /*if (backtouch && ballVy < 0)
    {
      ballVy = 0;
    }
    if (lefttouch && ballVx < 0)
    {
      ballVx = 0;
    }
    if (righttouch && ballVx > 0)
    {
      ballVx = 0;
    }*/
    Vector_Motion(int(ballVx), int(ballVy));
  }
  // float finalVx = (lineVx != 0) ? lineVx : ballVx + lineVx;
  // float finalVy = (lineVy != 0) ? lineVy : ballVy + lineVy;

  if (digitalRead(Pin) == 0)
  {
    backtouch = false;
  }
  if (digitalRead(Pin1) == 0)
  {
    lefttouch = false;
  }
  if (digitalRead(Pin2) == 0)
  {
    righttouch = false;
  }
  // delay(1000);
  if(dist_f<40 && ballVy > 0){
    ballVy = ballVy * 0.5;
  }
  Serial.print("Vy");Serial.println(ballVy);
  Serial.print("disf");Serial.println(dist_f);
    
}
void back()
{
  backtouch = true;
}
void left()
{
  lefttouch = true;
}
void right()
{
  righttouch = true;
}