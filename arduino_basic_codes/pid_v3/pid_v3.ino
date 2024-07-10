#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// BLDC motor & driver instances
BLDCMotor motor0 = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(9, 10, 11, 8);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(3, 5, 6, 7);

// MPU6050 instance
MPU6050 mpu;

const int potPin_p = A0;
const int potPin_i = A1;
const int potPin_d = A2;

// PID variables for motor 0
double setpoint0, input0, output0;
PID pid0(&input0, &output0, &setpoint0, 0, 0, 0, DIRECT);

// PID variables for motor 1
double setpoint1, input1, output1;
PID pid1(&input1, &output1, &setpoint1, 0, 0, 0, DIRECT);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  pinMode(potPin_p, INPUT);
  pinMode(potPin_i, INPUT);
  pinMode(potPin_d, INPUT);

  // Check MPU6050 connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialize BLDC drivers
  driver0.voltage_power_supply = 12;
  driver1.voltage_power_supply = 12;
  driver0.voltage_limit = 8;
  driver1.voltage_limit = 8;
  driver0.init();
  driver1.init();

  // Link the motors and drivers
  motor0.linkDriver(&driver0);
  motor1.linkDriver(&driver1);

  // Configure motors
  motor0.voltage_limit = 3;
  motor1.voltage_limit = 3;
  motor0.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;
  motor0.init();
  motor1.init();
  motor0.initFOC();
  motor1.initFOC();

  // PID configuration
  setpoint0 = 0; // Desired angle (0 degrees for balance)
  setpoint1 = 0; // Desired angle (0 degrees for balance)
  pid0.SetMode(AUTOMATIC);
  pid1.SetMode(AUTOMATIC);
  pid0.SetOutputLimits(-11, 11); // Motor output range
  pid1.SetOutputLimits(-11, 11); // Motor output range

  Serial.println("Setup complete");
}

void loop() {
  // Get MPU6050 angle
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Read potentiometer values and map to suitable PID ranges
  double pid_p = analogRead(potPin_p) / 10230.0 * 10; // Assuming PID P range 0-10
  double pid_i = analogRead(potPin_i) / 10230.0 * 20; // Assuming PID I range 0-10
  double pid_d = analogRead(potPin_d) / 10230.0 * 100; // Assuming PID D range 0-10

  // Apply new PID tunings
  // pid0.SetTunings(pid_p, pid_i, pid_d);
  // pid1.SetTunings(pid_p, pid_i, pid_d);
  pid0.SetTunings(pid_p, pid_i, 0.0043);
  pid1.SetTunings(pid_p, pid_i, 0.0043);

  // Calculate angle
  input0 = atan2(ay, az) * RAD_TO_DEG;
  input1 = input0; // Assuming both motors need to balance based on the same angle

  // Compute PID
  pid0.Compute();
  pid1.Compute();

  // Move motors based on PID output
  motor0.move(output0);
  motor1.move(output1);

  // Print angle and motor commands
  Serial.print("P: ");
  Serial.print(pid_p);
  Serial.print(" I: ");
  Serial.print(pid_i);
  Serial.print(" D: ");
  Serial.print(pid_d);
  Serial.print(" Angle: ");
  Serial.print(input0);
  Serial.print(" Output0: ");
  Serial.print(output0);
  Serial.print(" Output1: ");
  Serial.println(output1);

  delay(10); // Adjust the delay as needed
}