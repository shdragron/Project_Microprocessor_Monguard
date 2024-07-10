#include <SimpleFOC.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu(0x68); // 기본 I2C 주소는 0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long lastTime;
double dt;

double originalSetpoint = 185.5;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;

double input, output;
double Kp = 70;
double Kd = 2;
double Ki = 150;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// BLDC 모터 및 드라이버 인스턴스
BLDCMotor motor0 = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(9, 10, 11, 8);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(3, 5, 6, 7);

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C 클럭 (CPU가 8MHz인 경우 200kHz). 레오나르도는 250kHz로 측정되었습니다.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);  // 레오나르도 열거를 기다림, 다른 보드는 즉시 계속

  Serial.println(F("I2C 디바이스 초기화 중..."));
  mpu.initialize();

  Serial.println(F("디바이스 연결 테스트 중..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 연결 성공") : F("MPU6050 연결 실패"));

  // 드라이버 설정
  driver0.voltage_power_supply = 12;
  driver1.voltage_power_supply = 12;
  driver0.voltage_limit = 6;
  driver1.voltage_limit = 6;
  driver0.init();
  driver1.init();

  // 모터와 드라이버 연결
  motor0.linkDriver(&driver0);
  motor1.linkDriver(&driver1);

  // 모터 제한 설정
  motor0.voltage_limit = 3;   // [V]
  motor1.voltage_limit = 3;   // [V]
 
  // 오픈 루프 제어 설정
  motor0.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  // 모터 하드웨어 초기화
  motor0.init();
  motor1.init();

  // PID 초기화
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  Serial.println("모터 준비 완료!");

  lastTime = millis();
}

void loop() {
  // 1. 센서 데이터 읽기
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 2. 센서 데이터 처리
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 가속도 데이터를 각도로 변환 (단순화된 예제)
  double accelAngle = atan2(ay, az) * 180 / M_PI;
  double gyroRate = gx / 131.0; // 각속도(dps)

  // 각도 계산 (단순 칼만 필터 예제)
  double alpha = 0.98;
  input = alpha * (input + gyroRate * dt) + (1 - alpha) * accelAngle;

  // 3. PID 제어 계산
  pid.Compute();

  // 4. 모터 제어
  motor0.move(output);
  motor1.move(output);

  // 디버깅용 시리얼 출력
  Serial.print("Angle: ");
  Serial.print(input);
  Serial.print("\tOutput: ");
  Serial.println(output);
}
