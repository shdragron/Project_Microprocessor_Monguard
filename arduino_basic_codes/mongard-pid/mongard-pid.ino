#include <SimpleFOC.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;    // DMP 초기화가 성공하면 true로 설정
uint8_t mpuIntStatus;     // MPU의 실제 인터럽트 상태 바이트를 저장
uint8_t devStatus;        // 각 디바이스 작업 후 반환 상태 (0 = 성공, 0이 아니면 오류)
uint16_t packetSize;      // 예상 DMP 패킷 크기 (기본값은 42 바이트)
uint16_t fifoCount;       // 현재 FIFO에 있는 모든 바이트 수
uint8_t fifoBuffer[64];   // FIFO 저장 버퍼

Quaternion q;             // [w, x, y, z] 쿼터니언 컨테이너
VectorFloat gravity;      // [x, y, z] 중력 벡터
float ypr[3];             // [yaw, pitch, roll] yaw/pitch/roll 컨테이너 및 중력 벡터

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

volatile bool mpuInterrupt = false;  // MPU 인터럽트 핀이 높아졌는지 여부를 나타냄
void dmpDataReady() {
  mpuInterrupt = true;
}

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

  Serial.println(F("DMP 초기화 중..."));
  devStatus = mpu.dmpInitialize();

  // 다음 네 줄에 MPU6050 보정 값 입력:
  mpu.setXGyroOffset(4);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(17);
  mpu.setZAccelOffset(1958);

  if (devStatus == 0) {
    Serial.println(F("DMP 활성화 중..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("인터럽트 감지 활성화 (Arduino 외부 인터럽트 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP 준비 완료! 첫 번째 인터럽트 대기 중..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    Serial.print(F("DMP 초기화 실패 (코드 "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

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

  Serial.println("모터 준비 완료!");
}

void loop() {
  // 1. DMP 준비 확인
  if (!dmpReady) return;

  // 2. PID 제어 및 모터 움직임
  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute(); // PID 제어 계산

    // BLDC 모터 제어
    motor0.move(output);
    motor1.move(output);
  }

  // 3. 인터럽트 플래그 초기화 및 데이터 상태 확인
  mpuInterrupt = false; // 인터럽트 플래그 초기화
  mpuIntStatus = mpu.getIntStatus(); // MPU 인터럽트 상태 읽기
  fifoCount = mpu.getFIFOCount(); // FIFO 버퍼에 있는 데이터 수 읽기

  // 4. FIFO 데이터 읽기 및 처리
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // FIFO 오버플로우 처리
    mpu.resetFIFO(); // FIFO 버퍼 리셋
    Serial.println(F("FIFO 오버플로우!")); // 오류 메시지 출력
  } else if (mpuIntStatus & 0x02) {
    // FIFO 버퍼에서 데이터를 읽고 처리
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); // 충분한 데이터가 쌓일 때까지 대기
    mpu.getFIFOBytes(fifoBuffer, packetSize); // FIFO 버퍼에서 데이터를 읽어옴
    fifoCount -= packetSize; // 읽은 데이터만큼 FIFO 카운트 감소

    // 쿼터니언 데이터와 중력 벡터, yaw/pitch/roll 값 계산
    mpu.dmpGetQuaternion(&q, fifoBuffer); // 쿼터니언 데이터 가져오기
    mpu.dmpGetGravity(&gravity, &q); // 중력 벡터 가져오기
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // yaw, pitch, roll 값 계산

    // yaw, pitch, roll 값을 시리얼 모니터에 출력 (디버깅용)
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.println();

    // 5. 입력 값 설정 및 PID 제어에 사용
    input = ypr[1] * 180 / M_PI + 180;
  }
}
