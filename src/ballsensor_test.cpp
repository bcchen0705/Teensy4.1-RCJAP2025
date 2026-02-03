#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>


int ballVx;
int ballVy;


void setup() {
    Robot_Init();
}

void loop(){
    readBNO085Yaw();
    readBallCam();

    if(ballData.valid){    //有球
    Serial.print("Angle: "); Serial.println(ballData.angle);
    Serial.print("Dist: "); Serial.println(ballData.dist);

    //轉成弧度
    float moving_degree = ballData.angle;
    float ballspeed = constrain(map(ballData.dist, 40, 100, 20, 50),20, 50);
    float offset = 0;

    ballspeed = constrain(ballspeed, 20, 50);
    
    if(ballData.dist >= 50){
      moving_degree = ballData.angle;
      offset = 0;
    }
    else if(ballData.angle <= 105 && ballData.angle >= 75){
      ballspeed = 30;
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
    ballVx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
    ballVy = (int)round(ballspeed * sin(moving_degree * DtoR_const));

    Serial.printf("moving%f",moving_degree);Serial.print("");
    Serial.print("vx");Serial.println(ballVx);
    Serial.print("vy");Serial.println(ballVy);

    //誤差
    //float error = ballData.angle - gyroData.heading;

    //if(fabs(error) > 20.0f){
      //gyroData.control.robot_heading = ballData.angle;
    //}
    
    Vector_Motion(ballVx,ballVy);
    //delay(300);
  }
  else { //無球
    Serial.println("No Ball Detected");
    Vector_Motion(0,0);
  }
    /*readBNO085Yaw();
    ballsensor();
    Serial.println(ballData.dis);
    Serial.println(ballData.dir);
    Serial.println(" ");
    if(ballData.dir==3||ballData.dir==4){
        int speed = map(ballData.dis, 2, 9, 0, 30);
        Serial.print("speed");
        Serial.println(speed);
        Vector_Motion(0,speed);
    }
    else{
        Vector_Motion(0,0);
    }*/

}