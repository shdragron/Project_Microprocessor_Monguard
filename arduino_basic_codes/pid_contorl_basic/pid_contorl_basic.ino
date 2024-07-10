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

// PID variables for motor 0
double setpoint0, input0, output0;
PID pid0(&input0, &output0, &setpoint0, 2, 5, 1, DIRECT);

// PID variables for motor 1
double setpoint1, input1, output1;
PID pid1(&input1, &output1, &setpoint1, 2, 5, 1, DIRECT);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

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
  driver0.voltage_limit = 6;
  driver1.voltage_limit = 6;
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
  pid0.SetOutputLimits(-5, 5); // Adjust according to your system
  pid1.SetOutputLimits(-5, 5); // Adjust according to your system

  Serial.println("Setup complete");
}

void loop() {
  // Get MPU6050 angle
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

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
  Serial.print("Angle: ");
  Serial.print(input0);
  Serial.print(" Output0: ");
  Serial.print(output0);
  Serial.print(" Output1: ");
  Serial.println(output1);

  delay(10); // Adjust the delay as needed
}