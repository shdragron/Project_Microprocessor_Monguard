
#include <Servo.h>

Servo myservo1; // Left leg servo
Servo myservo2; // Right leg servo

int neutral1 = 1520;  // Calibrated neutral value for servo 1
int neutral2 = 1520;  // Calibrated neutral value for servo 2

int angleRange = 600;  // Maximum deviation from neutral

void setup() {
  myservo1.attach(9);
  myservo2.attach(10);
}


void loop() {
  walkForward();
  delay(2000);
  walkBackward();
  delay(2000);
}

void walkForward() {
  myservo1.writeMicroseconds(neutral1 + angleRange); // Move to max forward position
  myservo2.writeMicroseconds(neutral2 - angleRange); 
  delay(500); // Hold forward position for 500ms (adjust if needed)
  myservo1.writeMicroseconds(neutral1); // Return to neutral
  myservo2.writeMicroseconds(neutral2);
}

void walkBackward() {
  myservo1.writeMicroseconds(neutral1 - angleRange); // Move to max backward position
  myservo2.writeMicroseconds(neutral2 + angleRange);
  delay(500); // Hold backward position for 500ms (adjust if needed)
  myservo1.writeMicroseconds(neutral1); // Return to neutral
  myservo2.writeMicroseconds(neutral2);
}
