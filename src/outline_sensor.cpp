#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>


// Motor 1 Pins
#define pwmPin1 10    // PWM 控制腳
#define DIRA_1 11   // 方向控制腳1
#define DIRB_1 12

// Motor 2 Pins
#define pwmPin2 2    // PWM 控制腳
#define DIRA_2 3   // 方向控制腳1
#define DIRB_2 4

// Motor 3 Pins
#define pwmPin3 23    // PWM 控制腳
#define DIRA_3 36   // 方向控制腳1
#define DIRB_3 37

// Motor 4 Pins
#define pwmPin4 5   // PWM 控制腳
#define DIRA_4 6    // 方向控制腳1
#define DIRB_4 9

const int Pin=41;
const int Pin1=40;
const int Pin2=39;
volatile bool touch = false;
//void stop();

void setup() {
  pinMode(Pin,INPUT_PULLUP);
  pinMode(Pin1,INPUT_PULLUP);
  pinMode(Pin2,INPUT_PULLUP);
  Serial.begin(9600);
  Robot_Init();
  //attachInterrupt(digitalPinToInterrupt(Pin), stop,RISING);
  //attachInterrupt(digitalPinToInterrupt(Pin1), stop,RISING);
  //attachInterrupt(digitalPinToInterrupt(Pin2), stop,RISING);
}
void loop() {
  readBNO085Yaw();
  for(int i = 0; i <100; i++){
    SetMotorSpeed(1,10);
    SetMotorSpeed(2,10);
    SetMotorSpeed(3,10);
    SetMotorSpeed(4,10);
    delay(3000);
    SetMotorSpeed(1,-10);
    SetMotorSpeed(2,-10);
    SetMotorSpeed(3,-10);
    SetMotorSpeed(4,-10);
    delay(300);
    MotorStop();
    delay(300);
}
  //Serial.println(gyroData.pitch);

  //Serial.println("Pin");
  //Serial.println(digitalRead(Pin));
 /* Serial.println("Pin1");
  Serial.println(analogRead(Pin1));
  Serial.println(digitalRead(Pin1));*/

  //Serial.println("Pin2");
    
  //Serial.println(analogRead(Pin2));
  //Serial.println(digitalRead(Pin2));

  //Serial.println("Pin");
  //Serial.println(analogRead(Pin));



  //Serial.println("Pin1");
  //Serial.println(analogRead(Pin1));


  //Serial.println(analogRead(Pin2));
  

  //delay(10); // 每 0.5 秒讀一次

 /*if(touch){
  Serial.println("stop");*/
 }
 /*
 else{
  Serial.println("running");
 }

 delay(500);
}
void stop() {
  touch=true;
}*/