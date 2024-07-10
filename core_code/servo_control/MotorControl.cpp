#include "MotorControl.h"

void MotorControl::init() {
  DDRC |= (1 << motor1DirAPin) | (1 << motor1DirBPin) | (1 << motor2DirAPin) | (1 << motor2DirBPin); // Set direction pins as outputs
  DDRD |= (1 << motor1EnablePin) | (1 << motor2EnablePin);                                           // Set enable pins as outputs

  // Timer setup for PWM on pins 9 and 10 (adjust as needed for desired PWM frequency)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Fast PWM, 8-bit
  TCCR1B = (1 << CS11);                                  // Prescaler 8
}

void MotorControl::setSpeed(int motor, int speed) {
  if (motor == 1) {
    OCR1B = speed;
  } else if (motor == 2) {
    OCR1A = speed;
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
