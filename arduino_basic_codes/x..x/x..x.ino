#define DIN 13
#define CS 11
#define CLK 10

#define NUM_MATRICES 4
#define NUMBER_OF_ROWS 8

void setup() {
  Serial.begin(9600);

  pinMode(CLK, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(CS, OUTPUT);
  delay(100);

  for (uint8_t i = 0; i < NUM_MATRICES; i++) {
    write_Max7219(i, 0x09, 0x00); // Decode Mode Register – No decode for digits
    write_Max7219(i, 0x0A, 0x0F); // Intensity Register – 0x00 to 0x0F
    write_Max7219(i, 0x0B, 0x07); // Scan Limit Register – All Output Port Enable
    write_Max7219(i, 0x0C, 0x01); // SHUTDOWN Register – Normal Operation
    write_Max7219(i, 0x0F, 0x00); // Display Test Register – Display Test Mode
  }

  // Turn off all LEDs
  for (uint8_t i = 0; i < NUM_MATRICES; i++) {
    for (uint8_t j = 0; j < NUMBER_OF_ROWS; j++) {
      write_Max7219(i, j + 1, 0x00);
    }
  }
}

// "x" 표정 정의
const uint8_t arrowRight[8] = {
  0b10000001,
  0b01000010,
  0b00100100,
  0b00011000,
  0b00011000,
  0b00100100,
  0b01000010,
  0b10000001
};

// "x" 표정 정의
const uint8_t arrowLeft[8] = {
  0b10000001,
  0b01000010,
  0b00100100,
  0b00011000,
  0b00011000,
  0b00100100,
  0b01000010,
  0b10000001
};
void loop() {
  Serial.println("");

  // 첫 번째 매트릭스에 "x" 표정 표시
  for (uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
    write_Max7219(0, row + 1, arrowRight[row]); // 첫 번째 매트릭스
  }

  // 두 번째와 세 번째 매트릭스를 꺼둡니다
  for (uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
    write_Max7219(1, row + 1, 0x00); // 두 번째 매트릭스
    write_Max7219(2, row + 1, 0x00); // 세 번째 매트릭스
  }

  // 네 번째 매트릭스에 "x" 표정 표시
  for (uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
    write_Max7219(3, row + 1, arrowLeft[row]); // 네 번째 매트릭스
  }

  delay(2000); // 2초 간격으로 패턴 유지
}

void write_Max7219(uint8_t matrix, uint8_t address, uint8_t data) {
  digitalWrite(CS, LOW);
  for (uint8_t i = 0; i < NUM_MATRICES; i++) {
    if (i == matrix) {
      shiftOut(DIN, CLK, MSBFIRST, address);
      shiftOut(DIN, CLK, MSBFIRST, data);
    } else {
      shiftOut(DIN, CLK, MSBFIRST, 0);
      shiftOut(DIN, CLK, MSBFIRST, 0);
    }
  }
  digitalWrite(CS, HIGH);
}
