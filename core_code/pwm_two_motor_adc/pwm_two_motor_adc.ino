#include <Arduino.h>

// Pin definitions
#define MOTOR1_DIR_A_PIN 0x00  // MOTOR1 Direction pin A (Offset for PORTC)
#define MOTOR1_DIR_B_PIN 0x01  // MOTOR1 Direction pin B (Offset for PORTC)
#define MOTOR1_ENABLE_PIN 0x40 // MOTOR1 Enable pin (Offset for PORTD)

#define MOTOR2_DIR_A_PIN 0x02  // MOTOR2 Direction pin A (Offset for PORTC)
#define MOTOR2_DIR_B_PIN 0x03  // MOTOR2 Direction pin B (Offset for PORTC)
#define MOTOR2_ENABLE_PIN 0x20 // MOTOR2 Enable pin (Offset for PORTD)

#define POT_PIN 0x05            // Potentiometer input (Analog PIN A5)

void init_ADC() {
  ADMUX |= (0 << REFS1) | (1 << REFS0); // AVcc reference
  ADMUX &= ~(1 << ADLAR);               // Right adjust result
  ADCSRA |= (1 << ADEN);                // ADC enable
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
}

int read_ADC(uint8_t channel) {
  channel &= 0x07;                      // Ensure channel is between 0-7
  ADMUX = (ADMUX & 0xF8) | channel;     // Select ADC channel
  ADCSRA |= (1 << ADSC);                // Start ADC conversion
  while (ADCSRA & (1 << ADSC));         // Wait for conversion to complete
  return ADCW;                          // Read ADC value (10-bit)
}

void setup() {
  Serial.begin(9600);
  init_ADC();

  // Configure pins
  DDRC |= (1 << MOTOR1_DIR_A_PIN) | (1 << MOTOR1_DIR_B_PIN) | (1 << MOTOR2_DIR_A_PIN) | (1 << MOTOR2_DIR_B_PIN); // Set direction pins as outputs
  DDRD |= (1 << MOTOR1_ENABLE_PIN) | (1 << MOTOR2_ENABLE_PIN);                                                    // Set enable pins as outputs

  // Timer setup for PWM on pins 9 and 10 (adjust as needed for desired PWM frequency)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Fast PWM, 8-bit
  TCCR1B = (1 << CS11);                                  // Prescaler 8
}

void loop() {
  int pot_value = read_ADC(POT_PIN - A0);  
  int speed = map(pot_value, 0, 1023, 0, 255);

  // Set direction based on potentiometer value
  bool forward = (pot_value > 512);  

  // Motor 1 direction
  if (forward) {
    PORTC |= (1 << MOTOR1_DIR_A_PIN);
    PORTC &= ~(1 << MOTOR1_DIR_B_PIN);
  } else {
    PORTC &= ~(1 << MOTOR1_DIR_A_PIN);
    PORTC |= (1 << MOTOR1_DIR_B_PIN);
  }
  // Motor 1 speed
  //OCR1B = speed;
  int speed1 = 200;
  OCR1B = speed1;

  // Motor 2 direction
  if (forward) {
    PORTC |= (1 << MOTOR2_DIR_A_PIN);
    PORTC &= ~(1 << MOTOR2_DIR_B_PIN);
  } else {
    PORTC &= ~(1 << MOTOR2_DIR_A_PIN);
    PORTC |= (1 << MOTOR2_DIR_B_PIN);
  }
  // Motor 2 speed
  //OCR1A = speed;
  OCR1A = speed1;

  Serial.print("Potentiometer: ");
  Serial.print(pot_value);
  Serial.print("  Speed: ");
  Serial.print(speed);
  Serial.print("  Direction: ");
  Serial.println(forward ? "Forward" : "Backward");
  delay(100); 
}
