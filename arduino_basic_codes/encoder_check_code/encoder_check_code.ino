#include <SimpleFOC.h>

// MagneticSensorPWM(uint8_t _pinPWM, int _min, int _max)
// - _pinPWM:         the pin that is reading the pwm from magnetic sensor
// - _min_raw_count:  the minimal length of the pulse (in microseconds)
// - _max_raw_count:  the maximal length of the pulse (in microseconds)
MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 904);
void doPWM(){sensor.handlePWM();}

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  sensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}