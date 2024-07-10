#define PORTD_BUTTON_A  0x04 // PIN2
#define PORTD_BUTTON_B  0x08 // PIN3
#define PORTD_BUTTON_C  0x10 // PIN4
#define PORTD_BUTTON_D  0x20 // PIN5

#define X_CHANNEL 0x00 // ADC0
#define Y_CHANNEL 0x01 // ADC1

void init_ADC() {
  // Voltage reference: AVcc (REFS[1:0] == 01)
  ADMUX |= (0 << REFS1) | (1 << REFS0);

  // ADC Right Adjust Result
  ADMUX &= ~(1 << ADLAR);

  // ADC Enable and prescaler of 128
  // 16000000 / 128 = 125000
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

int read_ADC(uint8_t channel) {
  // ADC channel 0~7 사이 값 고르기
  channel &= 0x07;
  ADMUX = (ADMUX & 0xF8) | channel;

  // Start ADC
  ADCSRA |= (1 << ADSC);
  
  // Wait for conversion to complete
  while(ADCSRA & (1 << ADSC));

  // Read ADC value
  return ADC;
}

void setup() {
  Serial.begin(9600);

  // Initialize ADC;
  init_ADC();

  // set PIN2, PIN3, PIN4, PIN5 as activate internal pull-up resistors
  DDRD &= ~(PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
  PORTD |= (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);
}

void loop() {
  // Read joystick values
  int X = read_ADC(X_CHANNEL);
  int Y = read_ADC(Y_CHANNEL);

  // Read button states using PIND resister
  uint8_t buttonState = PIND & (PORTD_BUTTON_A | PORTD_BUTTON_B | PORTD_BUTTON_C | PORTD_BUTTON_D);

  Serial.print("joy stick  ");
  Serial.print("X:");
  Serial.print(X);
  Serial.print("  ");
  Serial.print("Y:");
  Serial.print(Y);
  Serial.print("   |");

  if (!(buttonState & PORTD_BUTTON_A)) {
    Serial.println("A: Yes  B: No  C: No  D: No");
  } else if (!(buttonState & PORTD_BUTTON_B)) {
    Serial.println("A: No  B: Yes  C: No  D: No");
  } else if (!(buttonState & PORTD_BUTTON_C)) {
    Serial.println("A: No  B: No  C: Yes  D: No");
    
  } else if (!(buttonState & PORTD_BUTTON_D)) {
    Serial.println("A: No  B: No  C: No  D: Yes");
  } else {
    Serial.println("A: No  B: No  C: No  D: No");
  }

  delay(50); // 50 milliseconds delay
}
