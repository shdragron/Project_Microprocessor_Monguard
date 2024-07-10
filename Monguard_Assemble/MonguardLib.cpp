#include "Arduino.h"
#include "MonguardLib.h"

Monguard::Monguard(int pin)
{
  _pin = pin;
}

void Monguard::begin()
{
  pinMode(_pin, OUTPUT);
}

void Monguard::dot()
{
  digitalWrite(_pin, HIGH);
  delay(250);
  digitalWrite(_pin, LOW);
  delay(250);
}

void Monguard::dash()
{
  digitalWrite(_pin, HIGH);
  delay(1000);
  digitalWrite(_pin, LOW);
  delay(250);
}

int Monguard::add(int a, int b) {
  return a + b;
}

int Monguard::subtract(int a, int b) {
  return a - b;
}
