#include <Arduino.h>

// 핀 정의
#define pinTrig 8
#define pinEcho 7
#define DIN 13
#define CS 12
#define CLK 11
#define SPEAKER_PIN 8
#define NUM_MATRICES 4
#define NUMBER_OF_ROWS 8

volatile bool triggerInterrupt = false;

// ">" 표현
const uint8_t arrowRight[8] = {
  0b00011000,
  0b00001100,
  0b00000110,
  0b00000011,
  0b00000011,
  0b00000110,
  0b00001100,
  0b00011000
};

// "<" 표현
const uint8_t arrowLeft[8] = {
  0b00011000,
  0b00110000,
  0b01100000,
  0b11000000,
  0b11000000,
  0b01100000,
  0b00110000,
  0b00011000
};

// "X" 표현
const uint8_t cross[8] = {
  0b10000001,
  0b01000010,
  0b00100100,
  0b00011000,
  0b00011000,
  0b00100100,
  0b01000010,
  0b10000001
};

// 멜로디 및 지속 시간 정의
const int melody[] = {
  // 추가하고 싶은 멜로디 음계
  262, 294, 330, 349, 392, 440, 494, 523
};

const int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 4, 4
};

void setup() {
  Serial.begin(9600);

  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  
  pinMode(CLK, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  delay(100);

  // LED 매트릭스 옵션 설정
  for (uint8_t i = 0; i < NUM_MATRICES; i++) {
    write_Max7219(i, 0x09, 0x00); // Decode Mode Register – No decode for digits
    write_Max7219(i, 0x0A, 0x0F); // Intensity Register – 0x00 to 0x0F
    write_Max7219(i, 0x0B, 0x07); // Scan Limit Register – All Output Port Enable
    write_Max7219(i, 0x0C, 0x01); // SHUTDOWN Register – Normal Operation
    write_Max7219(i, 0x0F, 0x00); // Display Test Register – Display Test Mode
  }

  // 초기 모든 LED 끄기
  for (uint8_t i = 0; i < NUM_MATRICES; i++) {
    for (uint8_t j = 0; j < NUMBER_OF_ROWS; j++) {
      write_Max7219(i, j + 1, 0x00);
    }
  }
}

double measureDistanceCm() {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(5);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  double duration = pulseIn(pinEcho, HIGH);
  double cm = (duration / 2) * 0.0343;
  return cm;
}

void loop() {
  double distance = measureDistanceCm();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance <= 10.0) {
    if (!triggerInterrupt) {
      triggerInterrupt = true;
      displayCross();
      playMelody();
    }
  } else {
    if (triggerInterrupt) {
      triggerInterrupt = false;
      displayArrows();
    }
  }

  delay(1000); // Delay for a second before next measurement
}

void displayArrows() {
  for (uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
    write_Max7219(0, row + 1, arrowRight[row]); // First Matrix
    write_Max7219(1, row + 1, 0x00); // Second Matrix
    write_Max7219(2, row + 1, 0x00); // Third Matrix
    write_Max7219(3, row + 1, arrowLeft[row]); // Fourth Matrix
  }
}

void displayCross() {
  for (uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
    write_Max7219(0, row + 1, cross[row]);
    write_Max7219(1, row + 1, 0x00);
    write_Max7219(2, row + 1, 0x00);
    write_Max7219(3, row + 1, cross[row]);
  }
}

void write_Max7219(uint8_t matrix, uint8_t address, uint8_t data) {
  digitalWrite(CS, LOW);
  for (uint8_t i = 0; i < NUM_MATRICES; i++) {
    if (i == matrix) {
      write_byte(address);
      write_byte(data);
    } else {
      write_byte(0);
      write_byte(0);
    }
  }
  digitalWrite(CS, HIGH);
}

void write_byte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    bool bit = (data & (1 << (7 - i)));
    digitalWrite(DIN, bit);
    digitalWrite(CLK, HIGH);
    digitalWrite(CLK, LOW);
  }
}

void playMelody() {
  // Iterate over the notes of the melody
  for (int thisNote = 0; thisNote < sizeof(melody) / sizeof(melody[0]); thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(SPEAKER_PIN, melody[thisNote], noteDuration);

    // To distinguish the notes, set a minimum time between them.
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(SPEAKER_PIN);
  }
}
