const int potPin = A1;
const int motor1Pin1 = 9;
const int motor1Pin2 = 10;
const int motor1Pin3 = 11;

const int motor2Pin1 = 6; // 모터 2에 대한 핀 설정
const int motor2Pin2 = 5; // 모터 2에 대한 핀 설정
const int motor2Pin3 = 3; // 모터 2에 대한 핀 설정

const int steps = 66;
int pwmSin[steps];

int currentStepA = 0;
int currentStepB = steps / 3; // 120도 위상차이
int currentStepC = 2 * (steps / 3); // 240도 위상차이

// 배열 초기화
void initPwmSin() {
  for (int i = 0; i < steps; i++) {
    pwmSin[i] = 127 + 127 * sin(2 * PI * i / steps); // 사인 함수 값을 0-255 사이로 변환
  }
}

void setup() {
  Serial.begin(9600);
  TCCR0B |= 0x03; // 타이머 설정(millis()와 delay()에 영향
  TCCR1B |= 0x01; // 9번 10번 핀의 PWM 주파수 설정 (31.25kHz)
  TCCR2B |= 0x01; // 11번과 3번 핀의 핀의 PWM 주파수 설정 (31.25kHz)
  ICR1 = 255; // PWM 해상도 8비트 설정

  pinMode(potPin, INPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1Pin3, OUTPUT);

  pinMode(motor2Pin1, OUTPUT); // 모터 2에 대한 핀 설정
  pinMode(motor2Pin2, OUTPUT); // 모터 2에 대한 핀 설정
  pinMode(motor2Pin3, OUTPUT); // 모터 2에 대한 핀 설정

  initPwmSin();
}

void loop() {
  move();
}

void move() {
  currentStepA = (currentStepA + 1) % steps;
  currentStepB = (currentStepA + steps / 3) % steps; // 120도 위상차이
  currentStepC = (currentStepA + 2 * (steps / 3)) % steps; // 240도 위상차이

  analogWrite(motor1Pin1, pwmSin[currentStepA]);
  analogWrite(motor1Pin2, pwmSin[currentStepC]);
  analogWrite(motor1Pin3, pwmSin[currentStepB]);

  analogWrite(motor2Pin1, pwmSin[currentStepA]); // 모터 2에 대한 PWM 제어
  analogWrite(motor2Pin2, pwmSin[currentStepC]); // 모터 2에 대한 PWM 제어
  analogWrite(motor2Pin3, pwmSin[currentStepB]); // 모터 2에 대한 PWM 제어

  // 모터 1과 모터 2에 대한 제어 코드 추가

  //Read pot value
  int sensorValue = analogRead(potPin);


  // Select ONLY ONE of the following lines for constant speed, speed control or position control:

  //This will give you constant speed, remember if you changed TCCR0B to 0x01, then delay(64000) = ~1 second
  //delay(5); 

  //This will give you open loop speed control with the potentiometer
  delay(sensorValue/10);

  //This will give you open loop position control with the potentiometer
  //currentStepA = sensorValue/5; //divide by a number to affect the ratio of pot position : motor position

}
