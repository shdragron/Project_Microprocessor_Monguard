// CURRENT SLAVE: 98d3,11,fc3cd1
// AT+BIND=98d3,11,fc3cd1
#include <SoftwareSerial.h>

SoftwareSerial mySerial(13, 12); // TX=2, RX=3 BLUETOOTH MODULE

void setup() {
  Serial.begin(9600);       // 시리얼 통신 시작, 전송 속도 9600
  mySerial.begin(38400);    // 블루투스 시리얼 통신 시작, 전송 속도 9600

  Serial.println("Enter AT commands:");
}

void loop() {
  // 블루투스에서 데이터 수신 시 시리얼 모니터에 출력
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
  
  // 시리얼 모니터에서 입력된 데이터를 블루투스로 전송  
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}
