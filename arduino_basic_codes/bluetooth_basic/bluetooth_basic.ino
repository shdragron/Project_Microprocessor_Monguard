#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 4); // TX=2, RX=3 BLUETOOTH MODULE
void setup() {
Serial.begin(9600);
Serial.println("Enter AT commands:");
mySerial.begin(38400);
}
void loop()
{
if (mySerial.available())
Serial.write(mySerial.read());
if (Serial.available())
mySerial.write(Serial.read());
}
