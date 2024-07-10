#ifndef MonguardLib_h
#define MonguardLib_h

#include "Arduino.h"

class Monguard
{
  public:
    Monguard(int pin);
    void begin();
    void dot();
    void dash();
    int add(int a, int b);
    int subtract(int a, int b);
  private:
    int _pin;
};

#endif
