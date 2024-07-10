#include <Wire.h>
#include <MPU9250.h>
#include <PID_v1.h>
#include <LMotorController.h>

// MPU9250 객체 생성
MPU9250 mpu;

double setpoint = 0;
double input, output;
double Kp = 50;
double Ki = 1.0;
double Kd = 1.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 0.6, 0.6);

void setup() {
    Wire.begin();
    Serial.begin(115200);
    delay(2000); // 전원 안정화를 위해 지연 시간 추가

    Serial.println("Starting setup...");

    // MPU9250 초기화 (AD0 핀이 GND에 연결된 경우 주소는 0x68)
    if (!mpu.setup(0x68)) {
        Serial.println("MPU9250 initialization failed!");
        while (1) {
            // 초기화 실패 시 경고 메시지 반복 출력
            Serial.println("MPU9250 initialization failed! Check connections and restart.");
            delay(1000);
        }
    }

    Serial.println("MPU9250 initialized successfully");

    // PID 설정
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
}

void loop() {
    mpu.update();

    // Quaternion을 사용하여 Yaw, Pitch, Roll 계산
    float ypr[3];
    ypr[0] = mpu.getYaw();
    ypr[1] = mpu.getPitch();
    ypr[2] = mpu.getRoll();

    // PID 입력을 Pitch 값으로 설정
    input = ypr[1];

    // PID 계산
    pid.Compute();

    // 모터 제어
    motorController.move(output, 20);  // MIN_ABS_SPEED를 20으로 설정

    Serial.print("ypr\t");
    Serial.print(ypr[0], 2);  // Yaw (Z)
    Serial.print("\t");
    Serial.print(ypr[1], 2);  // Pitch (Y)
    Serial.print("\t");
    Serial.println(ypr[2], 2);  // Roll (X)

    delay(10);
}
