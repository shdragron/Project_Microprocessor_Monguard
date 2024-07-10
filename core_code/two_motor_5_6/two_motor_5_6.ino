#include <Arduino.h>

// Pin definitions
#define MOTOR1_DIR_A_PIN 0x00  // MOTOR1 Direction pin A (Offset for PORTC)
#define MOTOR1_DIR_B_PIN 0x01  // MOTOR1 Direction pin B (Offset for PORTC)
#define MOTOR1_ENABLE_PIN 0x06 // MOTOR1 Enable pin (Offset for PORTD)

#define MOTOR2_DIR_A_PIN 0x02  // MOTOR2 Direction pin A (Offset for PORTC)
#define MOTOR2_DIR_B_PIN 0x03  // MOTOR2 Direction pin B (Offset for PORTC)
#define MOTOR2_ENABLE_PIN 0x05 // MOTOR2 Enable pin (Offset for PORTD)

void setup() {
  Serial.begin(9600);

  // Configure direction pins as outputs
  DDRC |= (1 << MOTOR1_DIR_A_PIN) | (1 << MOTOR1_DIR_B_PIN) | (1 << MOTOR2_DIR_A_PIN) | (1 << MOTOR2_DIR_B_PIN); 
  // Configure enable pins as outputs
  DDRD |= (1 << MOTOR1_ENABLE_PIN) | (1 << MOTOR2_ENABLE_PIN);

  // Timer setup for PWM on pins 5 and 6
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00); // Fast PWM, 8-bit
  TCCR0B = (1 << CS01);                                  // Prescaler 8

  // Set motors to move forward
  PORTC |= (1 << MOTOR1_DIR_A_PIN);
  PORTC &= ~(1 << MOTOR1_DIR_B_PIN);
  PORTC |= (1 << MOTOR2_DIR_A_PIN);
  PORTC &= ~(1 << MOTOR2_DIR_B_PIN);

  Serial.println("Enter speed (0-255): ");
}

void loop() {
  static int speed = 0;

  // Read speed input from serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    speed = input.toInt();
    speed = constrain(speed, 0, 255); // Ensure speed is between 0 and 255

    // Adjusted speed calculation for motor 1
    int motor1_speed = speed * 1.3;
    motor1_speed = constrain(motor1_speed, 0, 255); // Ensure motor1_speed does not exceed 255

    // Motor 1 speed
    OCR0A = motor1_speed;
    // Motor 2 speed
    OCR0B = speed;

    // Display the current speed
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print("  Motor 1 Speed: ");
    Serial.print(motor1_speed);
    Serial.println("  Motor 2 Speed: ");
    Serial.println(speed);
    Serial.println("Enter speed (0-255): ");
  }

  delay(100); 
}
