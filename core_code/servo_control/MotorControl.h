#ifndef MotorControl_h
#define MotorControl_h

#include <Arduino.h>

class MotorControl {
  public:
    void init();
    void setSpeed(int motor, int speed);
    void setDirection(int motor, char dir); // 수정된 함수 선언
    
  private:
    const int motor1DirAPin = 0x00;  // MOTOR1 Direction pin A (Offset for PORTC) : A0
    const int motor1DirBPin = 0x01;  // MOTOR1 Direction pin B (Offset for PORTC) : A1
    const int motor1EnablePin = 0x40; // MOTOR1 Enable pin (Offset for PORTD) : PIN6

    const int motor2DirAPin = 0x02;  // MOTOR2 Direction pin A (Offset for PORTC) : A2
    const int motor2DirBPin = 0x03;  // MOTOR2 Direction pin B (Offset for PORTC) : A3
    const int motor2EnablePin = 0x20; // MOTOR2 Enable pin (Offset for PORTD) : PIN5
};

#endif
