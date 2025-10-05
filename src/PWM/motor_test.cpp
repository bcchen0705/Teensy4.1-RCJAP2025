#include <Arduino.h>
#include "Robot.h"

void setup(){
  Robot_Init();
}

void loop(){
  MotorTest();
}

/*
const int pwmPin = 2;     // PWM 控制腳
const int dirPin1 = 3;    // 方向控制腳1
const int dirPin2 = 4; 

const int pwmPin1 = 23;    // PWM 控制腳
const int dirPin3 = 36;    // 方向控制腳1
const int dirPin4 =37;
const int maxSpeed = 150;  // 最大速度
const int stepDelay = 10;  // 加速/減速延遲(ms)

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // 一開始停住
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, 0);

   pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(dirPin4, OUTPUT);

  // 一開始停住
  digitalWrite(dirPin3, LOW);
  digitalWrite(dirPin4, LOW);
  analogWrite(pwmPin1, 0);
}


void loop() {
  MotorTest();
  // 正轉加速
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);
  digitalWrite(dirPin3, HIGH);
  digitalWrite(dirPin4, LOW);
  for(int speed = 0; speed <= maxSpeed; speed++) {
    analogWrite(pwmPin, speed);
    analogWrite(pwmPin1, speed);
    delay(stepDelay);
  }

  // 完全停住 1 秒
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, 0);
  digitalWrite(dirPin3, LOW);
  digitalWrite(dirPin4, LOW);
  analogWrite(pwmPin1,0);
  delay(1000);
  

  // 反轉加速
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(dirPin3, LOW);
  digitalWrite(dirPin4, HIGH);
  for(int speed = 0; speed <= maxSpeed; speed++) {
    analogWrite(pwmPin, speed);
    analogWrite(pwmPin1, speed);
    delay(stepDelay);
  }

  // 完全停住 1 秒
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, 0);
  digitalWrite(dirPin3, LOW);
  digitalWrite(dirPin4, LOW);
  analogWrite(pwmPin1, 0);
  delay(1000);
  
}
*/