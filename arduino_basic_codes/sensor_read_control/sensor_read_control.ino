// Open loop motor control example
#include <SimpleFOC.h>


// MagneticSensorPWM(uint8_t _pinPWM, int _min, int _max)
// - _pinPWM:         the pin that is reading the pwm from magnetic sensor
// - _min_raw_count:  the minimal length of the pulse (in microseconds)
// - _max_raw_count:  the maximal length of the pulse (in microseconds)
//MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 904);
MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 904);
void doPWM(){sensor.handlePWM();}


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

const int potPin = A1;  // INPUT pot control for speed or position

//target variable
float target_velocity = 10;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 3;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");

  pinMode(potPin, INPUT);

  // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);

  Serial.println("Sensor ready");
  
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  
  //Read pot value
  int sensorValue = analogRead(potPin); 
  
  //motor.move(target_velocity);
  //motor.move(sensorValue/30);

  //float speed = sensorValue/20;
  int speed = sensorValue/20;
  int check = 25;

  if(speed >= check){
    motor.move(speed-check);
  } else {
    motor.move(speed-check);
  }

 //Serial.println(sensorValue);
 //Serial.println(speed);

 // IMPORTANT - call as frequently as possible
  // update the sensor values 
  sensor.update();
  // display the angle and the angular velocity to the terminal
  // Serial.print("Speed set: ");
  // Serial.print(speed-check);
  Serial.print("\t Angle: ");
  Serial.print(sensor.getAngle());
  Serial.print("\t Velocity: ");
  Serial.println(sensor.getVelocity());

  // user communication
  command.run();
}
