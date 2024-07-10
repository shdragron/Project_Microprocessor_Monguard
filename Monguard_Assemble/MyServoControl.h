#ifndef MyServoControl_h
#define MyServoControl_h

#include <Arduino.h>

class MyServoControl {
public:
  MyServoControl();
  void begin();
  void walkForward(uint16_t speed);
  void walkBackward(uint16_t speed);
  void positionSet(uint16_t speed);
  void increaseAngle(uint16_t value, uint16_t speed);
  void decreaseAngle(uint16_t value, uint16_t speed);
  void tiltLeft(uint16_t value, uint16_t speed);
  void tiltRight(uint16_t value, uint16_t speed);
  void upDownTilt(uint16_t value, uint16_t speed);

private:
  const uint16_t NEUTRAL1 = 3925;
  const uint16_t NEUTRAL2 = 2280;
  const uint16_t ANGLE_RANGE = 200;

  uint16_t currentAngle1;
  uint16_t currentAngle2;

  void updateServo(uint16_t targetAngle1, uint16_t targetAngle2, uint16_t speed);
};

#endif
