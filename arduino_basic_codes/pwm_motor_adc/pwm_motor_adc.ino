#include <Arduino.h>

// Pin definitions
#define ENABLE_PIN  3   // PD3
#define DIR_F_PIN 7     // PD7
#define DIR_B_PIN 8     // PB0
#define BUTTON_PIN  12  // PB4
#define POT_PIN A5      // Analog PIN A5

enum MOTOR_DIRECTION { FOWARD = 0, BACKWARD = 1 };
enum MOTOR_DIRECTION Direction = FOWARD;

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
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_F_PIN, OUTPUT);
  pinMode(DIR_B_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Timer/Counter 2 setup for PWM on PD3 (OC2B)
  TCCR2A = (1 << WGM20) | (1 << WGM21);  // Fast PWM mode
  TCCR2B = (1 << CS22);                  // Prescaler 64
  TCCR2A |= (1 << COM2B1);               // Clear OC2B on compare match
  
  Serial.println("Ready to move");
}

void loop() {
  int pot_value = read_ADC(POT_PIN - A0); // Read potentiometer value (adjust channel number)
  int speed = map(pot_value, 0, 1023, 0, 255); // Map to 0-255 for PWM

  // Set the OCR2B register directly for PWM
  OCR2B = speed;

  // Read button state
  if (digitalRead(BUTTON_PIN) == LOW) {  // Button pressed
    delay(150);  // Debounce
    Direction = (Direction == FOWARD) ? BACKWARD : FOWARD;  // Toggle direction
    delay(100);  // Debounce
  }

  // Set direction
  if (Direction == FOWARD) {
    digitalWrite(DIR_F_PIN, HIGH);
    digitalWrite(DIR_B_PIN, LOW);
  } else {
    digitalWrite(DIR_F_PIN, LOW);
    digitalWrite(DIR_B_PIN, HIGH);
  }

  Serial.print("Direction: ");
  Serial.println(Direction);
  Serial.print("Speed: ");
  Serial.println(speed);
  delay(100); // Small delay to stabilize serial output
}
