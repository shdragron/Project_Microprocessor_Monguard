#include "MyServoControl.h"

// MyServoControl 객체 생성
MyServoControl myServo;

void setup() {
  // 시리얼 통신 초기화 (디버깅용)
  Serial.begin(9600);

  // 서보 모터 초기화
  myServo.begin();
  Serial.println("Servo initialized");

  // 서보 모터 중립 위치로 설정
  myServo.positionSet(10);
  Serial.println("Servo set to neutral position");
  delay(2000); // 2초 대기
}

void loop() {
  // 주기적으로 update 함수 호출
  myServo.update();

  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'U':
        myServo.walkForward(1);
        Serial.println("Servo moving UP");
        break;
      case 'D':
        myServo.walkBackward(1);
        Serial.println("Servo moving DOWN");
        break;
      case 'L':
        myServo.tiltLeft(300, 1);
        Serial.println("Servo tilted left");
        break;
      case 'R':
        myServo.tiltRight(300, 1);
        Serial.println("Servo tilted right");
        break;
      case 'N':
        myServo.positionSet(1);
        Serial.println("Servo set to neutral position");
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }
}
