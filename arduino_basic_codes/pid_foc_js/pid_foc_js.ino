#include <PID_v1.h>
//#include <LMotorController.h>
#include <SimpleFOC.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor0 = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver0 = BLDCDriver3PWM(9, 10, 11, 8);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(3, 5, 6, 7);

#define MIN_ABS_SPEED 10  // 모터의 최소 절대 속도

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

double originalSetpoint = 185.5; // 원래 설정점
double setpoint = originalSetpoint; // 설정점
double movingAngleOffset = 0.1; // 움직이는 각도 오프셋

double input, output;
int moveState = 0; // 이동 상태
double Kp = 70;
double Kd = 2;
double Ki = 150;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // PID 객체 생성

//double motorSpeedFactorLeft = 0.6; // 왼쪽 모터 속도 계수
//double motorSpeedFactorRight = 0.62; // 오른쪽 모터 속도 계수

// 모터 핀 설정
// int ENA = 3;
// int IN1 = 10;
// int IN2 = 9;
// int IN3 = 6;
// int IN4 = 5;
// int ENB = 11;

//LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;  // MPU 인터럽트 핀이 높아졌는지 여부를 나타냄
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // driver config
  // power supply voltage [V]
  driver0.voltage_power_supply = 11;
  driver1.voltage_power_supply = 11;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver0.voltage_limit = 11;
  driver0.init();
  driver1.voltage_limit = 11;
  driver1.init();
  // link the motor and the driver
  motor0.linkDriver(&driver0);
  motor1.linkDriver(&driver1);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor0.voltage_limit = 6;   // [V]
  motor1.voltage_limit = 6;   // [V]

  // open loop control config
  motor0.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor0.init();
  motor1.init();


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
  mpu.setXGyroOffset(212);
  mpu.setYGyroOffset(22);
  mpu.setZGyroOffset(-60);
  mpu.setZAccelOffset(1501);

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
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute();
    //motorController.move(output, MIN_ABS_SPEED);
    motor0.move(MIN_ABS_SPEED);
    motor1.move(MIN_ABS_SPEED);
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO 오버플로우!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if LOG_INPUT
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.println();
#endif
    input = ypr[1] * 180 / M_PI + 180;
  }
}