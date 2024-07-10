#include <SimpleFOC.h>

// BLDC motor instance
BLDCMotor motor = BLDCMotor(11);
// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// AS5048A PWM encoder instance
MagneticSensorPWM sensor = MagneticSensorPWM(2, 1000, 2000); // Adjust _min and _max raw counts based on AS5048A PWM range

// Potentiometer pin
const int potPin = A1;

// Function to handle PWM interrupt
void doPWM() {
  sensor.handlePWM();
}

void setup() {
  // Initialize serial communication for monitoring
  Serial.begin(115200);
  Serial.println("Motor and Sensor ready!");

  // Initialize AS5048A PWM encoder
  sensor.init();
  // Enable interrupt-based sensor reading
  sensor.enableInterrupt(doPWM); // Pass function pointer to handle PWM interrupt
  motor.linkSensor(&sensor);

  // Initialize BLDC driver
  driver.voltage_power_supply = 12; // Set power supply voltage (in volts)
  driver.init();
  motor.linkDriver(&driver);

  // Configure FOC modulation for torque control
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop; // Use open-loop velocity control

  // Initialize motor
  motor.init();

  // Set initial velocity target
  float target_velocity = 0.0;

  // Print initial setup messages
  Serial.println("Motor ready!");
  Serial.println("Control motor velocity with potentiometer");

  // Configure potentiometer pin
  pinMode(potPin, INPUT);
}

void loop() {
  // Read potentiometer value
  int sensorValue = analogRead(potPin);

  // Map potentiometer value to target velocity range (-20 to 20 rad/s)
  float target_velocity = map(sensorValue, 0, 1023, -20, 20);

  // Set target velocity for the motor
  motor.move(target_velocity);

  // Run motor control loop
  motor.loopFOC();

  // Display motor angle and velocity to serial monitor
  Serial.print("Angle: ");
  Serial.print(sensor.getAngle());
  Serial.print("\tVelocity: ");
  Serial.println(sensor.getVelocity());

  // Delay to control loop execution frequency
  delay(10); // Milliseconds
}
