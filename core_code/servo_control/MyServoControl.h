#ifndef MyServoControl_h
#define MyServoControl_h

#include <Arduino.h>

class MyServoControl {
public:
  MyServoControl();
  void begin();
  void walkForward(uint16_t speed = 10);
  void walkBackward(uint16_t speed = 10);
  void positionSet(uint16_t speed = 10);
  void increaseAngle(uint16_t value, uint16_t speed = 10);
  void decreaseAngle(uint16_t value, uint16_t speed = 10);
  void tiltLeft(uint16_t value, uint16_t speed = 10);
  void tiltRight(uint16_t value, uint16_t speed = 10);
  void upDownTilt(uint16_t value, uint16_t speed = 10);
  void update();

private:
  const uint16_t NEUTRAL1 = 3925;
  const uint16_t NEUTRAL2 = 2375;
  const uint16_t ANGLE_RANGE = 300;

  uint16_t currentAngle1;
  uint16_t currentAngle2;
  uint16_t targetAngle1;
  uint16_t targetAngle2;
  uint16_t step1;
  uint16_t step2;
  uint16_t speed;
  unsigned long lastUpdate;

  void moveServo();
};

#endif
