#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 4); // TX=2, RX=4 BLUETOOTH MODULE

void setup() {
  Serial.begin(9600);       // 시리얼 통신 시작, 전송 속도 9600
  mySerial.begin(9600);    // 블루투스 시리얼 통신 시작, 전송 속도 9600

  Serial.println("Enter AT commands:");
}

void loop() {
  // 블루투스에서 데이터 수신 시 시리얼 모니터에 출력
  if (mySerial.available()) {
    uint8_t read_value = mySerial.read();
    Serial.write(read_value);
  }
  
  // 시리얼 모니터에서 입력된 데이터를 블루투스로 전송
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}
