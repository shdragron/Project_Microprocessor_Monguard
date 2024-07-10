#include <Wire.h>

// MPU6050 레지스터 주소 정의
#define WHO_AM_I_MPU6050 0x75
#define ACCEL_CONFIG 0x1C
#define USER_CTRL 0x6A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define ACCEL_FULL_SCALE 0x00  // +/- 2g 설정

// 변수 선언
uint8_t acc_full_scale = 0x00;
float acc_scale;
char ax_str[10], ay_str[10], az_str[10];
uint8_t rawData[6];
int16_t accelCount[3];
char message[255] = {0};

// I2C 통신을 위한 함수
uint8_t transfer_I2C(uint8_t registerAddress, uint8_t data, bool isRead) {
  uint8_t response = 0x00;
  Wire.beginTransmission(0x68); // MPU6050 I2C 주소
  Wire.write(registerAddress);
  if (isRead) {
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1);
    response = Wire.read();
  } else {
    Wire.write(data);
    Wire.endTransmission();
  }
  return response;
}

// I2C 통신 비활성화 함수
void disable_i2c_comm() {
  uint8_t current_setting = transfer_I2C(USER_CTRL, 0x00, true);
  current_setting |= 0x10;  // I2C 통신 비활성화 설정
  transfer_I2C(USER_CTRL, current_setting, false);
}

// 가속도계 설정 함수
void setup_accel_scale(uint8_t scale) {
  uint8_t current_config = transfer_I2C(ACCEL_CONFIG, 0x00, true);
  current_config = (current_config & ~0x18) | (scale << 3);
  transfer_I2C(ACCEL_CONFIG, current_config, false);

  // 설정에 따라 scale 값 변경
  switch (scale) {
    case 0x00: acc_scale = 2.0 / 32768.0f; break;
    case 0x01: acc_scale = 4.0 / 32768.0f; break;
    case 0x02: acc_scale = 8.0 / 32768.0f; break;
    case 0x03: acc_scale = 16.0 / 32768.0f; break;
  }
}

void setup() {
  Serial.begin(38400);
  Wire.begin();
  
  // MPU6050 초기화 및 설정
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1 레지스터
  Wire.write(0x00); // 잠자기 모드 해제
  Wire.endTransmission();

  // 가속도계 설정
  setup_accel_scale(ACCEL_FULL_SCALE);
  delay(100);
}

void loop() {
  // 가속도 데이터 읽기
  Wire.beginTransmission(0x68);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);

  for (int i = 0; i < 6; i++) {
    rawData[i] = Wire.read();
  }

  // 데이터 변환
  accelCount[0] = (rawData[0] << 8) | rawData[1];
  accelCount[1] = (rawData[2] << 8) | rawData[3];
  accelCount[2] = (rawData[4] << 8) | rawData[5];

  float ax = accelCount[0] * acc_scale;
  float ay = accelCount[1] * acc_scale;
  float az = accelCount[2] * acc_scale;

  dtostrf(ax, 4, 2, ax_str);
  dtostrf(ay, 4, 2, ay_str);
  dtostrf(az, 4, 2, az_str);

  sprintf(message, "ax = %s, ay = %s, az = %s", ax_str, ay_str, az_str);
  Serial.println(message);

  delay(200);
}
