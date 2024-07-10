#include <SoftwareSerial.h>
#include <Arduino.h>

#define PORTD_BUTTON_A  0x04 // PIN2
#define PORTD_BUTTON_B  0x08 // PIN3
#define PORTD_BUTTON_C  0x10 // PIN4
#define PORTD_BUTTON_D  0x20 // PIN5
#define PORTB_BUTTON_E  0x01 // PIN8

#define X_CHANNEL 0x00 // ADC0
#define Y_CHANNEL 0x01 // ADC1

SoftwareSerial mySerial(11, 12); // TX=11, RX=12 BLUETOOTH MODULE

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
  return ADC;
}

struct DataPacket {
  char dir;
  int V_Left;
  int V_Right;
  char buttons[5]; // Increased to accommodate Button E
};

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  init_ADC();

  // Set pins 2, 3, 4, 5, 8 as inputs with internal pull-ups
  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |= (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  DDRB &= ~PORTB_BUTTON_E;
  PORTB |= PORTB_BUTTON_E;

  Serial.println("Enter AT commands:");
}

void loop() {
  // Read joystick values
  int X = read_ADC(X_CHANNEL);
  int Y = read_ADC(Y_CHANNEL);

  int steer = map(X, 0, 1023, -255, 255); // Steering split
  int speed;
  char dir;

  if (Y > 506) { 
    dir = 'F'; // Front
    speed = map(Y, 506, 1023, 0, 255);
  } else {
    dir = 'B'; // Back
    speed = map(Y, 0, 506, 255, 0);
  }

  int V_Left, V_Right;

  if (steer > 0) { // Turn right
    V_Left = speed;
    V_Right = speed - steer;
  } else { // Turn left
    V_Left = speed + steer;
    V_Right = speed;
  }

  V_Left = constrain(V_Left, 0, 255);
  V_Right = constrain(V_Right, 0, 255);

  // Read button states using PIND and PINB registers and format into a single byte
  char buttons[5] = {'0', '0', '0', '0', '0'}; // Increased to accommodate Button E
  if (!(PIND & PORTD_BUTTON_A)) {
    buttons[0] = 'A';
  }
  if (!(PIND & PORTD_BUTTON_B)) {
    buttons[1] = 'B';
  }
  if (!(PIND & PORTD_BUTTON_C)) {
    buttons[2] = 'C';
  }
  if (!(PIND & PORTD_BUTTON_D)) {
    buttons[3] = 'D';
  }
  if (!(PINB & PORTB_BUTTON_E)) {
    buttons[4] = 'E';
  }

  // Send data packet: [dir, V_Left, V_Right, buttonState]
  DataPacket dataPacket;
  dataPacket.dir = dir;
  dataPacket.V_Left = V_Left;
  dataPacket.V_Right = V_Right;
  dataPacket.buttons[0] = buttons[0];
  dataPacket.buttons[1] = buttons[1];
  dataPacket.buttons[2] = buttons[2];
  dataPacket.buttons[3] = buttons[3];
  dataPacket.buttons[4] = buttons[4];

  mySerial.write((uint8_t *)&dataPacket, sizeof(dataPacket));

  // Print debug information
  Serial.print("Dir: ");
  Serial.print(dir);
  Serial.print(" V_Left: ");
  Serial.print(V_Left);
  Serial.print(" V_Right: ");
  Serial.print(V_Right);
  Serial.print(" Buttons: ");
  Serial.println(buttons);

  delay(50);
}
