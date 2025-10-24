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
        Vector_Motion(0, 0);  // 停止動作
    }
    else{
        float ballDegree = ballDegreelist[ballData.dir];
        float offset = 0;

        Serial.print("balldir=");Serial.println(ballData.dir);
        Serial.print("ballDegree=");Serial.println(ballDegree);
        Serial.print("ballData.dis=");Serial.println(ballData.dis);
        Serial.print("exp");Serial.println(exp(-0.75*ballData.dis));

        map(ballData.dis,0,12,0,10); 

        if((ballDegree >= 22.5 && ballDegree < 87.5) || (ballDegree > 92.5 && ballDegree <= 167.5) ){
            offset = 90*(exp(-0.75*(ballData.dis-4)));            
            offset = (ballDegree > 90 ) ? offset : -offset;
            Serial.print("offset="); Serial.println(offset);
        }

        else if( ballDegree == 87.5 || ballDegree == 92.5){
            offset = 0;
        }

        else if(ballDegree > 167.5 && ballDegree < 385.5){
            offset = 90*(exp(-0.75*(ballData.dis-7)));            
            offset = (ballDegree < 270 ) ? offset : -offset;    
            Serial.print("offset="); Serial.println(offset);           
        }

        float moving_Degree = ballDegree + offset;
        Serial.print("moving_Degree="); Serial.println(moving_Degree);
        
        ballVx = 20 * cos(moving_Degree* DtoR_const );
        ballVy = 20 * sin( moving_Degree * DtoR_const );

    float sumX = 0, sumY = 0;
    int count = 0;
    Serial.print("lineData.state=");
    Serial.println(lineData.state, BIN);

    for (int i = 0; i < 18; i++) {
        bool detected = ((lineData.state & (1UL << i)) == 0); // 0 表有線
        Serial.print("Sensor "); Serial.print(i);
        Serial.print(": "); Serial.println(detected ? "ON line" : "OFF line");
        if (detected) {
            float deg = linesensorDegreelist[i] + 180;
            if (deg >= 360) deg -= 360;

            //Serial.print("  deg +180 = "); Serial.println(deg);

            sumX += cos(deg * DtoR_const);
            sumY += sin(deg * DtoR_const);
            count++;
        }
    }
    if (count > 0) {
        float lineDegree = atan2(sumY, sumX) * RtoD_const;
        if (lineDegree < 0) lineDegree += 360;

        Serial.print("sumX="); Serial.print(sumX);
        Serial.print(", sumY="); Serial.print(sumY);
        Serial.print(", average lineDegree="); Serial.println(lineDegree);

        float speed = 20;
        lineVx = speed * cos(lineDegree*DtoR_const);
        lineVy = speed * sin(lineDegree*DtoR_const);
        Serial.print("lineVx="); Serial.print(lineVx);
        Serial.print("lineVy="); Serial.println(lineVy);
    }
    else{
        lineVx=0;
        lineVy=0;
    }
    float finalVx = ballVx + lineVx;
    float finalVy = ballVy + lineVy;

    if (backtouch && finalVy < 0) {finalVy = 0;}
    if (lefttouch && finalVx < 0)  {finalVx = 0;}
    if (righttouch && finalVx > 0) {finalVx = 0;}

    Vector_Motion(int(finalVx), int(finalVy));

    Serial.print("finalVx="); Serial.print(finalVx);
    Serial.print("finalVy="); Serial.println(finalVy);


    if(digitalRead(Pin)==0){
        backtouch = false;}
    if(digitalRead(Pin1)==0){
        lefttouch = false;}
    if(digitalRead(Pin2)==0){
        righttouch = false;}
    delay(1000);
}
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