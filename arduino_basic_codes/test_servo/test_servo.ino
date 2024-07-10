#define NEUTRAL1 3925 // Calibrated neutral value for servo 1 (Left) (green, blue, orange) (in microseconds)
#define NEUTRAL2 2375 // Calibrated neutral value for servo 2 (Right) (black, red, orange) (in microseconds)
//1980 front leg servo high

#define ANGLE_RANGE 300// Maximum deviation from neutral

void setup() {
  Serial.begin(9600);

  // Configure Timer/Counter1 for Fast PWM mode
  cli(); // Disable global interrupts
  DDRB |= (1 << PB1) | (1 << PB2); // Set OC1A (PB1) and OC1B (PB2) as output

  // Configure Timer/Counter1
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, 10-bit
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, Prescaler 8

  ICR1 = 40000; // Top value for 50Hz PWM (20ms period)

  sei(); // Enable global interrupts

  // Initialize the PWM values to neutral position
  OCR1A = NEUTRAL1; // Timer counts in microseconds
  OCR1B = NEUTRAL2; // Timer counts in microseconds
}

void loop() {
//  walkForward();
  
  walkBackward();
//  delay(2000);
//  position_set();
}

void position_set() {
  OCR1A = NEUTRAL1;
  OCR1B = NEUTRAL2;
}

void walkForward() {
  OCR1A = NEUTRAL1 + ANGLE_RANGE; // Move to max forward position
  OCR1B = NEUTRAL2 - ANGLE_RANGE;
}

void walkBackward() {
  OCR1A = NEUTRAL1 - ANGLE_RANGE; // Move to max backward position
  OCR1B = NEUTRAL2 + ANGLE_RANGE;
}
