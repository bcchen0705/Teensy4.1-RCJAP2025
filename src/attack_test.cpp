#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#define RtoD_const 57.2958

const int Pin=41;
const int Pin1=40;
const int Pin2=39;

volatile bool backtouch = false;
volatile bool lefttouch = false;
volatile bool righttouch = false;

float ballVx = 0;
float ballVy = 0;
float lineVx = 0;
float lineVy = 0;

const float EMERGENCY_THRESHOLD = 80; 
float prev_lineDegree = -1;    
bool emergency = false;

void back();
void left();
void right();

void setup() {
pinMode(Pin,INPUT_PULLUP);
pinMode(Pin1,INPUT_PULLUP);
pinMode(Pin2,INPUT_PULLUP);

Robot_Init();
Serial.begin(9600);

attachInterrupt(digitalPinToInterrupt(Pin), back,RISING);
attachInterrupt(digitalPinToInterrupt(Pin1), left,RISING);
attachInterrupt(digitalPinToInterrupt(Pin2), right,RISING);

}

void loop(){
    readBNO085Yaw();
    ballsensor();
    linesensor();
    if(ballData.dir == 255){
        Serial.println("No ball detected");
        ballVx = 0;
        ballVy = 0;
    }
    else{
        float ballDegree = ballDegreelist[ballData.dir];
        float offset = 0;

        Serial.print("balldir=");Serial.println(ballData.dir);
        Serial.print("ballDegree=");Serial.println(ballDegree);
        Serial.print("ballData.dis=");Serial.println(ballData.dis);
        Serial.print("exp");Serial.println(exp(-0.55*(ballData.dis-7)));
        //delay(2000);     

        if( ballDegree == 87.5 || ballDegree == 92.5){
            offset = 0;
        }

        else {
            
            double offsetRatio = exp(-0.55*(ballData.dis-7));
            offsetRatio = (exp(-0.55*(ballData.dis-7)) > 1 ) ? 1 : offsetRatio ;
            offset = 100*offsetRatio;      
            offset = (ballDegree > 90 ) ? offset : -offset;
            offset = (ballDegree < 270 ) ? offset : -offset;
            Serial.print("offset="); Serial.println(offset);

        }

        float moving_Degree = ballDegree + offset;
        Serial.print("moving_Degree="); Serial.println(moving_Degree);
        
        float ballspeed = map(ballData.dis,7,12,15,35);
        ballspeed = constrain(ballspeed,15,35);
        Serial.print("BallSpeed="); Serial.println(ballspeed);
        ballVx = ballspeed * cos(moving_Degree* DtoR_const );
        ballVy = ballspeed * sin( moving_Degree * DtoR_const );
    }
    float sumX = 0, sumY = 0;
    int count = 0;
    //Serial.print("lineData.state=");
    //Serial.println(lineData.state, BIN);

    for (int i = 0; i < 18; i++) {
        bool detected = ((lineData.state & (1UL << i)) == 0); // 0 表有線
        //Serial.print("Sensor "); Serial.print(i);
        //Serial.print(": "); Serial.println(detected ? "ON line" : "OFF line");
        if (detected) {
            float deg = linesensorDegreelist[i] + 180;
            if (deg >= 360) deg -= 360;

            //Serial.print("  deg +180 = "); Serial.println(deg);

            sumX += cos(deg * DtoR_const);
            sumY += sin(deg * DtoR_const);
            count++;
        }
    }
    float diff = 0;
    if (count > 0) {
        float lineDegree = atan2(sumY, sumX) * RtoD_const;
        if (lineDegree < 0) lineDegree += 360;
        showLine();
        //Serial.print("sumX="); Serial.print(sumX);
        //Serial.print(", sumY="); Serial.print(sumY);
        //Serial.print(", average lineDegree="); Serial.println(lineDegree);
        if(prev_lineDegree >= 0){
        diff = fabs(fmod((lineDegree - prev_lineDegree + 180), 360) - 180);
        
        }
        prev_lineDegree = lineDegree;
        
       
        float finalDegree;
        if(diff > EMERGENCY_THRESHOLD){
            finalDegree = lineDegree;
            showEmergency();
        }
        else{
            finalDegree = fmod(lineDegree + 180, 360);
        }
        float speed = 40;
        lineVx = speed * cos(finalDegree*DtoR_const);
        lineVy = speed * sin(finalDegree*DtoR_const);
        
        //Serial.print("lineVx="); Serial.print(lineVx);
        //Serial.print("lineVy="); Serial.println(lineVy);
    }
    else{
        lineVx=0;
        lineVy=0;
        showStart();
    }
    float finalVx = ballVx + lineVx;
    float finalVy = ballVy + lineVy;

    if (backtouch && finalVy < 0) {finalVy = 0;}
    if (lefttouch && finalVx < 0)  {finalVx = 0;}
    if (righttouch && finalVx > 0) {finalVx = 0;}

    Vector_Motion(int(finalVx), int(finalVy));

    //Serial.print("finalVx="); Serial.print(finalVx);
    //Serial.print("finalVy="); Serial.println(finalVy);


    if(digitalRead(Pin)==0){
        backtouch = false;}
    if(digitalRead(Pin1)==0){
        lefttouch = false;}
    if(digitalRead(Pin2)==0){
        righttouch = false;}
    //delay(1000);
}
void back() {
backtouch = true;
}
void left() {
lefttouch = true;
}
void right() {
righttouch = true;
}