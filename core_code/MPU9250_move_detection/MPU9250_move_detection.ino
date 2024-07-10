#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

float lastX = 0;
float threshold = 0.8; // 변화량 임계값 (원하는 값으로 조정)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
}

void loop() {
  if (mpu.update()) {
    float currentX = mpu.getAccX();

    float deltaX = abs(currentX - lastX);

    if (deltaX > threshold) {
      Serial.println("good");
    }

    Serial.print("X-axis acceleration: ");
    Serial.println(currentX);

    lastX = currentX;
  }
  delay(100); // 데이터 갱신 주기 (원하는 값으로 조정)
}
