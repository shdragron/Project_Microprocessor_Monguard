#ifndef MotorControl_h
#define MotorControl_h

#include <Arduino.h>

class MotorControl {
  public:
    void init();
    void setSpeed(int motor, int speed);
    void setDirection(int motor, char dir); // 수정된 함수 선언
    
  private:
    const int motor1DirAPin = 0x00;  // MOTOR1 Direction pin A: A0
    const int motor1DirBPin = 0x01;  // MOTOR1 Direction pin B: A1
    const int motor1EnablePin = 0x06; // MOTOR1 Enable pin: PIN6

    const int motor2DirAPin = 0x02;  // MOTOR2 Direction pin A: A2
    const int motor2DirBPin = 0x03;  // MOTOR2 Direction pin B: A3
    const int motor2EnablePin = 0x05; // MOTOR2 Enable pin: PIN5
};

#endif
