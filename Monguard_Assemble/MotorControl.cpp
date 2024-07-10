#include "MotorControl.h"

void MotorControl::init() {
  DDRC |= (1 << motor1DirAPin) | (1 << motor1DirBPin) | (1 << motor2DirAPin) | (1 << motor2DirBPin); 
  DDRD |= (1 << motor1EnablePin) | (1 << motor2EnablePin);                                           

  // Timer setup
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00); // Fast PWM, 8-bit
  TCCR0B = (1 << CS01);                                  // Prescaler 8
}

void MotorControl::setSpeed(int motor, int speed) {
  if (motor == 1) {
    OCR0A = speed;
  } else if (motor == 2) {
    OCR0B = speed;
  }
}

void MotorControl::setDirection(int motor, char dir) {
  if (motor == 1) {
    if (dir == 'F') {
      PORTC |= (1 << motor1DirAPin);
      PORTC &= ~(1 << motor1DirBPin);
    } else if (dir == 'B') {
      PORTC &= ~(1 << motor1DirAPin);
      PORTC |= (1 << motor1DirBPin);
    } else if (dir == 'N') {
      PORTC &= ~(1 << motor1DirAPin);
      PORTC &= ~(1 << motor1DirBPin);
    }
  } else if (motor == 2) {
    if (dir == 'F') {
      PORTC |= (1 << motor2DirAPin);
      PORTC &= ~(1 << motor2DirBPin);
    } else if (dir == 'B') {
      PORTC &= ~(1 << motor2DirAPin);
      PORTC |= (1 << motor2DirBPin);
    } else if (dir == 'N') {
      PORTC &= ~(1 << motor2DirAPin);
      PORTC &= ~(1 << motor2DirBPin);
    }
  }
}
