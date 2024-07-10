#include "MyServoControl.h"

MyServoControl::MyServoControl() : currentAngle1(NEUTRAL1), currentAngle2(NEUTRAL2) {}


void MyServoControl::begin() {
  // Serial.begin(9600);

  // 타이머/카운터1을 Fast PWM 모드로 설정
  cli(); // 전역 인터럽트 비활성화
  DDRB |= (1 << PB1) | (1 << PB2);

  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, 10-bit
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, 프리스케일러 8

  ICR1 = 40000; // 50Hz PWM(20ms 주기)

  sei(); // 전역 인터럽트 활성화

  // 중립 위치로 PWM 값을 초기화
  OCR1A = NEUTRAL1; // 타이머 카운트는 마이크로초 단위
  OCR1B = NEUTRAL2; // 타이머 카운트는 마이크로초 단위
}

// 지정된 속도로 위치 설정
void MyServoControl::positionSet(uint16_t speed) {
  updateServo(NEUTRAL1, NEUTRAL2, speed);
}

// 앞으로
void MyServoControl::walkForward(uint16_t speed) {
  updateServo(NEUTRAL1 + ANGLE_RANGE, NEUTRAL2 - ANGLE_RANGE, speed);
}

// 뒤로
void MyServoControl::walkBackward(uint16_t speed) {
  updateServo(NEUTRAL1 - ANGLE_RANGE, NEUTRAL2 + ANGLE_RANGE, speed);
}

void MyServoControl::increaseAngle(uint16_t value, uint16_t speed) {
  updateServo(constrain(currentAngle1 + value, NEUTRAL1 - ANGLE_RANGE, NEUTRAL1 + ANGLE_RANGE),
              constrain(currentAngle2 + value, NEUTRAL2 - ANGLE_RANGE, NEUTRAL2 + ANGLE_RANGE),
              speed);
}

void MyServoControl::decreaseAngle(uint16_t value, uint16_t speed) {
  updateServo(constrain(currentAngle1 - value, NEUTRAL1 - ANGLE_RANGE, NEUTRAL1 + ANGLE_RANGE),
              constrain(currentAngle2 - value, NEUTRAL2 - ANGLE_RANGE, NEUTRAL2 + ANGLE_RANGE),
              speed);
}

// 오른쪽으로 기울기
void MyServoControl::tiltRight(uint16_t value, uint16_t speed) {
  updateServo(constrain(currentAngle1 + value, NEUTRAL1 - ANGLE_RANGE, NEUTRAL1 + ANGLE_RANGE),
              constrain(currentAngle2 + value, NEUTRAL2 - ANGLE_RANGE, NEUTRAL2 + ANGLE_RANGE),
              speed);
}

// 왼쪽으로 기울기
void MyServoControl::tiltLeft(uint16_t value, uint16_t speed) {
  updateServo(constrain(currentAngle1 - value, NEUTRAL1 - ANGLE_RANGE, NEUTRAL1 + ANGLE_RANGE),
              constrain(currentAngle2 - value, NEUTRAL2 - ANGLE_RANGE, NEUTRAL2 + ANGLE_RANGE),
              speed);
}

// 상하
void MyServoControl::upDownTilt(uint16_t value, uint16_t speed) {
  updateServo(constrain(currentAngle1 + value, NEUTRAL1 - ANGLE_RANGE, NEUTRAL1 + ANGLE_RANGE),
              constrain(currentAngle2 - value, NEUTRAL2 - ANGLE_RANGE, NEUTRAL2 + ANGLE_RANGE),
