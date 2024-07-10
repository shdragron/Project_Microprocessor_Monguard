#include <Servo.h>

Servo myservo1;
Servo myservo2;

int pulseWidth = 1500; // Initial pulse width

void setup() {
  Serial.begin(9600);
  myservo1.attach(9);
  myservo2.attach(10);
}

void loop() {
  myservo1.writeMicroseconds(pulseWidth);
  myservo2.writeMicroseconds(pulseWidth);

  Serial.print("Pulse Width: ");
  Serial.println(pulseWidth);

  delay(2000); // Wait for 2 seconds to observe

  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'u') {
      pulseWidth += 10; // Increase pulse width
    } else if (command == 'd') {
      pulseWidth -= 10; // Decrease pulse width
    }
  }
}
