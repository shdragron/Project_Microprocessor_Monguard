// Open loop motor control example
 #include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor0 = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(9, 10, 11, 8);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(3, 5, 6, 7);


// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

//target variable
float target_position = 30;

const int potPin = A1;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_position, cmd); }
// motor0
void doLimit0(char* cmd) { command.scalar(&motor0.voltage_limit, cmd); }
void doVelocity0(char* cmd) { command.scalar(&motor0.velocity_limit, cmd); }
// motor1
void doLimit1(char* cmd) { command.scalar(&motor1.voltage_limit, cmd); }
void doVelocity1(char* cmd) { command.scalar(&motor1.velocity_limit, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver0.voltage_power_supply = 12;
  driver1.voltage_power_supply = 12;
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
  // currnet = resistance*voltage, so try to be well under 1Amp
  motor0.voltage_limit = 6;   // [V]
  motor1.voltage_limit = 6;   // [V]
  // limit/set the velocity of the transition in between 
  // target angles
  motor0.velocity_limit = 5; // [rad/s] cca 50rpm
  motor1.velocity_limit = 5; // [rad/s] cca 50rpm
  // open loop control config
  motor0.controller = MotionControlType::angle_openloop;
  motor1.controller = MotionControlType::angle_openloop;

  // init motor hardware
  motor0.init();
  motor1.init();

  // add target command T
  command.add('T', doTarget, "target angle");
  command.add('L', doLimit0, "voltage limit");
  command.add('V', doLimit0, "movement velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target position [rad]");

  pinMode(potPin, INPUT);

  _delay(1000);
}

void loop() {
  // open  loop angle movements
  // using motor.voltage_limit and motor.velocity_limit
  // angles can be positive or negative, negative angles correspond to opposite motor direction
  
  //Read pot value
  float sensorValue = analogRead(potPin); 
  
  float angle = sensorValue / 142;
  
  //motor.move(target_position);
  motor0.move(angle);
  motor1.move(angle);
  Serial.println(angle);
  
  // user communication
  command.run();
}