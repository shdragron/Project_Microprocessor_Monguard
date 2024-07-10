#ifndef faceControl_h
#define faceControl_h

#include <Arduino.h>
#include <SPI.h>

class faceControl {
public:
  faceControl(uint8_t DIN, uint8_t CS, uint8_t CLK, uint8_t NUM_MATRICES);
  void begin();
  void setFace(String expression);
  void clearDisplay();
  void winking();

private:
  uint8_t _DIN, _CS, _CLK, _NUM_MATRICES;
  const uint8_t NUMBER_OF_ROWS = 8;

  unsigned long lastUpdateTime;
  bool isWinking;

  // Declare expression arrays in the header file
  static const uint8_t normalEyes[8];
  static const uint8_t squintEyesLeft[8];
  static const uint8_t squintEyesRight[8];
  static const uint8_t surprisedEyes[8];
  static const uint8_t winkLeft[8];
  static const uint8_t winkRight[8];
  static const uint8_t sadEyes[8];
  static const uint8_t angryEyesLeft[8];
  static const uint8_t angryEyesRight[8];

  static const uint8_t smileMouth1[8];
  static const uint8_t smileMouth2[8];
  static const uint8_t flatMouth1[8];
  static const uint8_t flatMouth2[8];
  static const uint8_t openMouth1[8];
  static const uint8_t openMouth2[8];
  static const uint8_t winkMouth1[8];
  static const uint8_t winkMouth2[8];
  static const uint8_t sadMouth1[8];
  static const uint8_t sadMouth2[8];

  void write_Max7219(uint8_t matrix, uint8_t address, uint8_t data);
  void write_byte(uint8_t data);
  void normalface();
  void squintface();
  void surprisedface();
  void winkface();
  void sadface();
  void angryface();
};

#endif
