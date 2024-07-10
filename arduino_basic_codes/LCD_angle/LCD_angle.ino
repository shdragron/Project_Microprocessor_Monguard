#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <LiquidCrystal_I2C.h>

MPU6050 mpu;
int16_t accY, accZ;
float accAngle;

LiquidCrystal_I2C lcd(0x27, 20, 4);  // LCD의 I2C 주소와 크기 설정

void setup() {
  Wire.begin();
  mpu.initialize();
  Serial.begin(9600);
  lcd.init();  // LCD 초기화
  lcd.backlight();  // LCD 백라이트 켜기

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 connected");
  } else {
    Serial.println("MPU6050 connection failed");
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 failed");
    while (1);  // 연결 실패 시 멈춤
  }

  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();
}

void loop() {
  mpu.getAcceleration(NULL, &accY, &accZ);
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;

  if (!isnan(accAngle)) {
    Serial.println(accAngle);
    lcd.setCursor(0, 0);
    lcd.print("YAngle: ");
    lcd.println(accY);
    lcd.print("ZAngle: ");
    lcd.println(accZ);
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Invalid Angle");
  }

  delay(100);
}
