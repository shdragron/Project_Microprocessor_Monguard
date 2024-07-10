#include <Arduino.h>
#include "faceControl.h"

#define DIN 2
#define CS 3
#define CLK 4
#define NUM_MATRICES 4
#define NUMBER_OF_ROWS 8

faceControl face(DIN, CS, CLK, NUM_MATRICES);

void setup() {
  Serial.begin(9600);
  face.begin();
}

void loop() {
  face.setFace("normal");
  delay(2000);
  face.setFace("squint");
  delay(2000);
  face.setFace("surprised");
  delay(2000);
  face.setFace("wink");
  face.winking();
  delay(2000);
  face.setFace("sad");
  delay(2000);
  face.setFace("angry");
  delay(2000);
  face.setFace("clear");
  delay(2000);
}
