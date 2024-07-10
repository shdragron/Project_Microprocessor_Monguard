#include <Arduino.h>

void setup() {
  Serial.begin(9600);

  // 디지털 핀 0~13을 입력으로 설정하고 풀업 저항 활성화
  for (int pin = 0; pin <= 13; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }

  // 아날로그 핀 A0~A5를 입력으로 설정하고 풀업 저항 활성화
  for (int pin = A0; pin <= A5; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }
}

void loop() {
  // 디지털 핀 0~13의 상태를 읽고 버튼이 눌렸는지 확인
  for (int pin = 0; pin <= 13; pin++) {
    if (digitalRead(pin) == LOW) {
      Serial.print("Button pressed on pin ");
      Serial.println(pin);
      delay(200); // 디바운싱 방지
    }
  }

  // 아날로그 핀 A0~A5의 상태를 읽고 버튼이 눌렸는지 확인
  for (int pin = A0; pin <= A5; pin++) {
    if (digitalRead(pin) == LOW) {
      Serial.print("Button pressed on pin A");
      Serial.println(pin - A0);  // A0부터 시작하므로 A0을 빼서 0~5 값으로 출력
      delay(200); // 디바운싱 방지
    }
  }
}
