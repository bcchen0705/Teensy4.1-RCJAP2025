#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>



const int Pin=41;
const int Pin1=40;
const int Pin2=39;
volatile bool lefttouch = false;
volatile bool righttouch = false;
volatile bool backtouch = false;
void backsensor();
void leftsensor();
void rightsensor();


void setup() {
    pinMode(Pin,INPUT_PULLUP);
    pinMode(Pin1,INPUT_PULLUP);
    pinMode(Pin2,INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(Pin), backsensor,RISING);
    attachInterrupt(digitalPinToInterrupt(Pin1), leftsensor,RISING);
    attachInterrupt(digitalPinToInterrupt(Pin2), rightsensor,RISING);

    Robot_Init();
    
    Serial.begin(9600);
    }

void loop() {
  /*Serial.println("Pin");
  Serial.println(digitalRead(Pin));
  Serial.println("Pin1");
  Serial.println(digitalRead(Pin1));
  Serial.println("Pin2");
  Serial.println(digitalRead(Pin2));
  delay(500); // 每 0.5 秒讀一次
  */
 readBNO085Yaw();
 if(backtouch==true){
    Serial.println("forward");
    Vector_Motion(0, 10);
    if(digitalRead(Pin)==0){
        Serial.println("stop");
        Vector_Motion(0, 0);
        backtouch=false;
    }
 }
 else if (lefttouch==true){
    Serial.println("left");
    Vector_Motion(10, 0);
    if(digitalRead(Pin1)==0){
        Serial.println("stop");
        Vector_Motion(0, 0);
        lefttouch=false;
    }
 }
 else if (righttouch==true){
    Serial.println("right");
    Vector_Motion(-10, 0);
    if(digitalRead(Pin2)==0){
        Serial.println("stop");
        Vector_Motion(0, 0);
        righttouch=false;
    }
 }
 else{
    Serial.println("stop");
    Vector_Motion(0, 0);
 }
}
void backsensor() {
  backtouch=true;
}
void leftsensor() {
  lefttouch=true;
}
void rightsensor() { 
  righttouch=true;
}