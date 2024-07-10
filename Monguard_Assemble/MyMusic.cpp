#include "MyMusic.h"

MyMusic::MyMusic(int pin) {
  _pin = pin;
  pinMode(_pin, OUTPUT);
  _melodyIndex = 0;
  _previousMillis = 0;
}

void MyMusic::playMelody() {
  startMelody();
}

void MyMusic::startMelody() {
  _melodyIndex = 0;
  _previousMillis = millis();
}

void MyMusic::update() {
  if (_melodyIndex >= 7) {
    noTone(_pin);
    return;
  }

  unsigned long currentMillis = millis();
  
  // 원래 코드의 기본 템포 (밀리초 단위로 4분음표의 지속 시간)
  int tempo = 150; // 125 밀리초 = 원래의 빠른 속도
  
  // 현재 음의 지속 시간 계산
  int noteDuration = tempo * (8.0 / smileNoteDurations[_melodyIndex]);

  if (currentMillis - _previousMillis >= noteDuration * 1.30) {
    _previousMillis = currentMillis;

    tone(_pin, smileMelody[_melodyIndex], noteDuration);
    _melodyIndex++;
  }
}
