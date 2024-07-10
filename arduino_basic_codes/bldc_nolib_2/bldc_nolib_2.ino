const int potPin = A1;
const int motorPin1 = 9;
const int motorPin2 = 10;
const int motorPin3 = 11;

const int steps = 49;
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
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);

  initPwmSin();
}

void loop() {
  move();
}

void move() {
  currentStepA = (currentStepA + 1) % steps;
  currentStepB = (currentStepA + steps / 3) % steps; // 120도 위상차이
  currentStepC = (currentStepA + 2 * (steps / 3)) % steps; // 240도 위상차이

  analogWrite(motorPin1, pwmSin[currentStepA]);
  analogWrite(motorPin2, pwmSin[currentStepC]);
  analogWrite(motorPin3, pwmSin[currentStepB]);

  //Following send data to PLX-DAQ macro for Excel
  /*Serial.print("DATA,");
  Serial.print(pwmSin[currentStepA]);
  Serial.print(","); 
  Serial.print(pwmSin[currentStepB]);
  Serial.print(","); 
  Serial.println(pwmSin[currentStepC]);
  */

  //Read pot value
  int sensorValue = analogRead(potPin);


  // Select ONLY ONE of the following lines for constant speed, speed control or position control:

  //This will give you constant speed, remember if you changed TCCR0B to 0x01, then delay(64000) = ~1 second
  //delay(5); 

  //This will give you open loop speed control with the potentiometer
  //delay(sensorValue/10);
  //delay(10);

  //This will give you open loop position control with the potentiometer
  currentStepA = sensorValue/5; //divide by a number to affect the ratio of pot position : motor position



}
