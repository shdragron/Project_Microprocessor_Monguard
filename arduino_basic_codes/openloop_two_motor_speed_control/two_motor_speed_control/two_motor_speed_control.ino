// Open loop motor control example
#include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor0 = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver0 = BLDCDriver3PWM(9, 10, 11, 8);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(3, 5, 6, 7);
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10,11, 8);


// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

const int potPin = A1;  // INPUT pot control for speed or position

//target variable
// float target_velocity = 10;

// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget0(char* cmd) { command.scalar(&target_velocity, cmd); }
// void doLimit0(char* cmd) { command.scalar(&motor0.voltage_limit, cmd); }
// void doTarget1(char* cmd) { command.scalar(&target_velocity, cmd); }
// void doLimit1(char* cmd) { command.scalar(&motor1.voltage_limit, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver0.voltage_power_supply = 12;
  driver1.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver0.voltage_limit = 12;
  driver0.init();
  driver1.voltage_limit = 12;
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

  // add target command T
  // command.add('T', doTarget0, "target velocity");
  // command.add('L', doLimit0, "voltage limit");

  // Serial.begin(115200);
  // Serial.println("Motor ready!");
  // Serial.println("Set target velocity [rad/s]");

  pinMode(potPin, INPUT);
  
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  
  //Read pot value
  float sensorValue = analogRead(potPin); 
  
  //motor.move(target_velocity);
  //motor.move(sensorValue/30);

  float speed = sensorValue/20;

  if(speed >= 25){
    motor0.move(speed-25);
    motor1.move(speed-25);
  } else {
    motor0.move(speed-25);
    motor1.move(speed-25);
  }

 //Serial.println(sensorValue);
 Serial.println(speed);

  // user communication
  //command.run();
}
