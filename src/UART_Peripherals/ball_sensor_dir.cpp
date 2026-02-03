#include <Arduino.h>
#include <Robot.h>
#include <math.h>

struct BallCamData{
    uint16_t angle = 65535;
    uint16_t dist = 65535;
    bool valid = false;
}; BallCamData ballCamData;

int ballvx = 0;
int ballvy = 0;


void setup() {
  Serial.begin(115200);
  Robot_Init();
}

void loop(){
  readBNO085Yaw();
    static uint8_t buffer[6];
    static uint8_t index = 0;
    
    // 假設每次 loop 都檢查 UART
    while(Serial4.available()){
        uint8_t b = Serial4.read();

        // 等待起始符
        if(index == 0 && b != 0xCC) continue;

        buffer[index++] = b;

        // 收到完整封包
        if(index == 6){
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
                ballCamData.angle = buffer[1] | (buffer[2] << 8);
                ballCamData.dist = buffer[3] | (buffer[4] << 8);

                // 判斷是否有效
                if(ballCamData.angle != 65535 && ballCamData.dist != 65535){
                    ballCamData.valid = true;
                     Serial.print("Angle: "); Serial.println(ballCamData.angle);
                    //Serial.print("Distance: "); Serial.println(ballCamData.dist);
                    float angle_rad = ballCamData.angle * DtoR_const;
                    ballvx = (int)round(10.0f * cos(angle_rad));
                    ballvy = (int)round(10.0f * sin(angle_rad));
                    Serial.print("vx");Serial.println(ballvx);
                    Serial.print("vy");Serial.println(ballvy);
                    Vector_Motion(ballvx,ballvy);
                } else {
                    ballCamData.valid = false;
                    Serial.println("No Ball (invalid data)");
                    MotorStop();
                }
            } 
            else {
                ballCamData.valid = false;
                Serial.println("No Ball (bad packet)");
                MotorStop();
            }
            index = 0;  // reset buffer
        }
    }
}