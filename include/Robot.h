#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
//Buttons
#define BUTTON_LEFT  31
#define BUTTON_RIGHT 30
//Outter Linesensor
/*TODO*/

//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Motors
//outter left pwm = 2, dir = 3,4
//inner left pwm = 23, dir = 36,37
//outter right pwm = 5, dir = 6,9
//inner right pwm = 10, dir = 11,12

#define PWM_1 999
#define DIRA_1 999
#define DIRB_1 999
#define PWM_2 999
#define DIRA_2 999
#define DIRB_2 999
#define PWM_3 999
#define DIRA_3 999
#define DIRB_3 999
#define PWM_4 999
#define DIRA_4 999
#define DIRB_4 999
//Kicker
#define kicker_charge 999
#define kicker_release 999


//Sensors
struct GyroData {float heading = 0.0; bool valid = false;} gyroData;
struct LineData {uint32_t state = 999; bool valid = false;} lineData;
struct BallData {uint8_t dir = 255; uint8_t dis = 255; bool valid = false;} ballData;

/*Sensors Part*/
/*
void readBNO085Yaw(HardwareSerial &serial) {
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];


  while (serial.available() >= PACKET_SIZE) {
    buffer[0] = serial.read();
    if (buffer[0] != 0xAA) continue;
    buffer[1] = serial.read();
    if (buffer[1] != 0xAA) continue;


    for (int i = 2; i < PACKET_SIZE; i++) buffer[i] = serial.read();


    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    gyroData.valid = true;
    gyroData.heading = yaw_raw * 0.01f; // 轉成度
    break;
  }
}*/

void Robot_Init(){
    Serial2.begin(115200);//Gyro Sensor
    Serial3.begin(115200);//OnBoard Maix Bit
    Serial4.begin(115200);//Ball Sensor
    Serial5.begin(115200);//Line Sensor
    Serial6.begin(115200);//OnBoard ESP32
    Serial7.begin(115200);//
    Serial8.begin(115200);//
    //Menu Buttons
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);
    //OnBoard LED
    pinMode(LED_BUILTIN, OUTPUT);
    //Motor 1
    pinMode(PWM_1, OUTPUT);
    pinMode(DIRA_1, OUTPUT);
    pinMode(DIRA_1, OUTPUT);
    //Motor 2
    pinMode(PWM_2, OUTPUT);
    pinMode(DIRA_2, OUTPUT);
    pinMode(DIRB_2, OUTPUT);
    //Motor 3
    pinMode(PWM_3, OUTPUT);
    pinMode(DIRA_3, OUTPUT);
    pinMode(DIRB_3, OUTPUT);
    //Motor 4
    pinMode(PWM_4, OUTPUT);
    pinMode(DIRA_4, OUTPUT);
    pinMode(DIRB_4, OUTPUT);
    //Kicker
    pinMode(kicker_charge, OUTPUT);
    pinMode(kicker_release, OUTPUT);
    
    Wire.begin();
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
}

void readBNO085Yaw() {
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  while (Serial2.available() >= PACKET_SIZE) {
    buffer[0] = Serial2.read();
    if (buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if (buffer[1] != 0xAA) continue;
    for (int i = 2; i < PACKET_SIZE; i++) buffer[i] = Serial2.read();
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    gyroData.valid = true;
    gyroData.heading = yaw_raw * 0.01f; // 轉成度
    break;
  }
}

void ballsensor(){
  uint8_t buffer_index = 0;
  uint8_t buffer[3];
  ballData.valid = false;
  while (Serial4.available()) {
    uint8_t b = Serial4.read();
    if (buffer_index == 0 && b != 0xAA) return; // wait for start
    buffer[buffer_index++] = b;

    if (buffer_index == 3) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[2] == 0xEE) {
        uint8_t temp = buffer[1];
        ballData.valid = true;
        ballData.dir = (temp & 0x0F);
        ballData.dis = (temp & 0xF0) >> 4;
      }
    }
  }
}

void linesensor(){
  uint8_t buffer_index = 0;
  uint8_t buffer[7];
  lineData.valid = false;
  while (Serial5.available()) {
    uint8_t b = Serial5.read();
    if (buffer_index == 0 && b != 0xAA) return; // wait for start
    buffer[buffer_index++] = b;
    if (buffer_index == 7) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[0] == 0xAA && buffer[6] == 0xEE) {
        lineData.state = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16);
        uint8_t checksum = (buffer[2] + buffer[3] + buffer[4]) & 0xFF;
        if (checksum == buffer[5]) {
            lineData.valid = true;
            Serial.print("sensors: ");
            Serial.println(lineData.state, BIN);
        }
      }
    }
  }
}
/*Actuators Part*/

   
void showStart() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println("Start");
  display.display();
}


void showRunScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println("Run");
  display.display();
}


void showSensors(float gyro, int light, int ball) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("Gyro: ");  display.println(gyro);
  display.setCursor(0, 15); display.print("Light: "); display.println(light);
  display.setCursor(0, 30); display.print("Ball: ");  display.println(ball);
  display.display();
}



void SetMotorSpeed(uint8_t port, int8_t speed) {
  // Clamp speed to -100..100
  speed = constrain(speed, -100, 100);

  // Map absolute speed to 0..255 for PWM
  int pwmVal = map(abs(speed), 0, 100, 0, 255);
  switch (port) {
    case 1: // Motor 1
      analogWrite(PWM_1, pwmVal);
      if (speed > 0) {
        digitalWrite(DIRA_1, HIGH);
        digitalWrite(DIRA_2, LOW);
      } else if (speed < 0) {
        digitalWrite(DIRA_1, LOW);
        digitalWrite(DIRA_2, HIGH);
      } else {
        digitalWrite(DIRA_1, LOW);
        digitalWrite(DIRA_2, LOW);
      }
      break;

    case 2: // Motor 2
      analogWrite(PWM_2, pwmVal);
      if (speed > 0) {
        digitalWrite(DIRA_2, HIGH);
        digitalWrite(DIRB_2, LOW);
      } else if (speed < 0) {
        digitalWrite(DIRA_2, LOW);
        digitalWrite(DIRB_2, HIGH);
      } else {
        digitalWrite(DIRA_2, LOW);
        digitalWrite(DIRB_2, LOW);
      }
      break;

    case 3: // Motor 3
      analogWrite(PWM_3, pwmVal);
      if (speed > 0) {
        digitalWrite(DIRA_3, HIGH);
        digitalWrite(DIRB_3, LOW);
      } else if (speed < 0) {
        digitalWrite(DIRA_3, LOW);
        digitalWrite(DIRB_3, HIGH);
      } else {
        digitalWrite(DIRA_3, LOW);
        digitalWrite(DIRB_3, LOW);
      }
      break;

    case 4: // Motor 4
      analogWrite(PWM_4, pwmVal);
      if (speed > 0) {
        digitalWrite(DIRA_4, HIGH);
        digitalWrite(DIRB_4, LOW);
      } else if (speed < 0) {
        digitalWrite(DIRA_4, LOW);
        digitalWrite(DIRB_4, HIGH);
      } else {
        digitalWrite(DIRA_4, LOW);
        digitalWrite(DIRB_4, LOW);
      }
      break;

    default:
      // Invalid motor port
      break;
  }
}

void MotorStop(){
  SetMotorSpeed(1, 0);
  SetMotorSpeed(2, 0);
  SetMotorSpeed(3, 0);
  SetMotorSpeed(4, 0);  
}

bool MotorTest(){
  //left rotation
  SetMotorSpeed(1, 10);
  SetMotorSpeed(2, 10);
  SetMotorSpeed(3, 10);
  SetMotorSpeed(4, 10);  
  delay(1000);
  SetMotorSpeed(1, 10);
  SetMotorSpeed(2, 10);
  SetMotorSpeed(3, 10);
  SetMotorSpeed(4, 10); 
  delay(1000);
  //right rotation
  return true;
}

//remove
void RobotFKControl(){
  ;
}

void RobotIKControl(int8_t vx, int8_t vy, float omega){
  int8_t p1 = -vx + vy + omega;
  int8_t p2 = -vx - vy + omega;
  int8_t p3 = vx - vy + omega;
  int8_t p4 = vx + vy + omega;
  SetMotorSpeed(1, p1);
  SetMotorSpeed(2, p2);
  SetMotorSpeed(3, p3);
  SetMotorSpeed(4, p4);
}

