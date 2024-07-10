#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;
int16_t accX, accY, accZ;
float rollAngle, pitchAngle;

void setup() {
  Wire.begin();
  mpu.initialize();
  Serial.begin(9600);
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  mpu.getAcceleration(&accX, &accY, &accZ);

  // Pitch 각도 계산 (X축과 Z축을 사용)
  pitchAngle = atan2(accX, accZ) * RAD_TO_DEG;

  // 90도를 기준으로 각도 변환 (피치)
  float transformedPitchAngle = pitchAngle;
  if (transformedPitchAngle < -90) transformedPitchAngle += 360;
  if (transformedPitchAngle > 270) transformedPitchAngle -= 360;

  if (!isnan(transformedPitchAngle)) {
    Serial.print("Pitch Angle: ");
    Serial.println(transformedPitchAngle);
  } else {
    Serial.println("Invalid Angle");
  }

  delay(100); // 적절한 딜레이 추가
}
