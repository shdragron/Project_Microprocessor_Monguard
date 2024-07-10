#include <Arduino.h>
#include "MyMusic.h"

#define BUZZER_PIN 11

MyMusic music(BUZZER_PIN);

void setup() {
  Serial.begin(9600);
}

void loop() {
  music.playSmileMelody();
  delay(2000);

  music.playCryMelody();
  delay(2000);
}
