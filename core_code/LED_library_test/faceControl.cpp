#include "faceControl.h"

//Expression definitions
const uint8_t faceControl::normalEyes[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b11111111,
    0b11111111,
    0b00000000,
    0b00000000,
    0b00000000
};

const uint8_t faceControl::angryEyesLeft[8] = {
    0b00000000,
    0b00011000,
    0b00001100,
    0b00000110,
    0b00000011,
    0b00000001,
    0b00000000,
    0b00000000
};

const uint8_t faceControl::angryEyesRight[8] = {
    0b00000000,
    0b00011000,
    0b00110000,
    0b01100000,
    0b11000000,
    0b10000000,
    0b00000000,
    0b00000000
};

const uint8_t faceControl::squintEyesLeft[8] = {
    0b00011000,
    0b00001100,
    0b00000110,
    0b00000011,
    0b00000011,
    0b00000110,
    0b00001100,
    0b00011000
};

const uint8_t faceControl::squintEyesRight[8] = {
    0b00011000,
    0b00110000,
    0b01100000,
    0b11000000,
    0b11000000,
    0b01100000,
    0b00110000,
    0b00011000
};

const uint8_t faceControl::surprisedEyes[8] = {
    0b00000000,
    0b00111100,
    0b01100110,
    0b01000010,
    0b01000010,
    0b01100110,
    0b00111100,
    0b00000000
};

const uint8_t faceControl::winkLeft[8] = {
    0b00011000,
    0b00001100,
    0b00000110,
    0b01111111,
    0b01111111,
    0b00000110,
    0b00001100,
    0b00011000
};

const uint8_t faceControl::winkRight[8] = {
    0b00000000,
    0b00111100,
    0b01100110,
    0b01000010,
    0b01000010,
    0b01100110,
    0b00111100,
    0b00000000
};

const uint8_t faceControl::sadEyes[8] = {
    0b00000000,
    0b11111111,
    0b01100110,
    0b11001100,
    0b01100110,
    0b00110011,
    0b01100110,
    0b00000000
};

const uint8_t faceControl::smileMouth2[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00110000,
    0b11100000,
    0b00000000
};

const uint8_t faceControl::smileMouth1[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00001100,
    0b00000111,
    0b00000000
};

const uint8_t faceControl::flatMouth1[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00011111,
    0b00000000
};

const uint8_t faceControl::flatMouth2[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b11111000,
    0b00000000
};

const uint8_t faceControl::openMouth1[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000011,
    0b00000100,
    0b00000011,
    0b00000000
};

const uint8_t faceControl::openMouth2[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b11000000,
    0b00100000,
    0b11000000,
    0b00000000
};

const uint8_t faceControl::winkMouth1[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000011,
    0b00000010,
    0b00000001,
    0b00000000
};

const uint8_t faceControl::winkMouth2[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b11000000,
    0b01000000,
    0b10000000,
    0b00000000
};

const uint8_t faceControl::sadMouth1[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000001,
    0b00000011,
    0b00000110,
    0b00001100,
    0b00000000
};

const uint8_t faceControl::sadMouth2[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b10000000,
    0b11000000,
    0b01100000,
    0b00110000,
    0b00000000
};

faceControl::faceControl(uint8_t DIN, uint8_t CS, uint8_t CLK, uint8_t NUM_MATRICES)
    : _DIN(DIN), _CS(CS), _CLK(CLK), _NUM_MATRICES(NUM_MATRICES) {}

unsigned long previousMillis = 0;
const long interval = 500; // Interval for blinking (500ms)
bool isWinking = false;
bool winkState = false;

void faceControl::begin() {
    pinMode(_CLK, OUTPUT);
    pinMode(_DIN, OUTPUT);
    pinMode(_CS, OUTPUT);
    delay(100);

    for (uint8_t i = 0; i < _NUM_MATRICES; i++) {
        write_Max7219(i, 0x09, 0x00);
        write_Max7219(i, 0x0A, 0x0F);
        write_Max7219(i, 0x0B, 0x07);
        write_Max7219(i, 0x0C, 0x01);
        write_Max7219(i, 0x0F, 0x00);
    }

    clearDisplay();
}

void faceControl::clearDisplay() {
    for(uint8_t i = 0; i < _NUM_MATRICES; i++) {
        for(uint8_t j = 0; j < NUMBER_OF_ROWS; j++) {
            write_Max7219(i, j + 1, 0x00);
        }
    }
}

void faceControl::setFace(String expression) {
    if (expression == "normal") {
        normalface();
    } else if (expression == "squint") {
        squintface();
    } else if (expression == "surprised") {
        surprisedface();
    } else if (expression == "wink") {
        isWinking = true;
        lastUpdateTime = millis();
        winkface();
//        update();
    } else if (expression == "sad") {
        sadface();
    } else if (expression == "angry") {
        angryface();
    }else {
        clearDisplay();
    }

}


void faceControl::write_Max7219(uint8_t matrix, uint8_t address, uint8_t data) {
  digitalWrite(_CS, LOW);
  for (uint8_t i = 0; i < _NUM_MATRICES; i++) {
    if (i == matrix) {
      write_byte(address);
      write_byte(data);
    } else {
      write_byte(0);
      write_byte(0);
    }
  }
  digitalWrite(_CS, HIGH);
}

void faceControl::write_byte(uint8_t data){
    for(uint8_t i = 0; i < 8; i++) {
        bool bit = (data & (1 << (7 - i)));
///        if(verbose)
//   /         Serial.print(bit);
        digitalWrite(_DIN, bit);
        digitalWrite(_CLK, HIGH);
        digitalWrite(_CLK, LOW);
    }
}

void faceControl::normalface() {
    for(uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
        write_Max7219(0, row + 1, normalEyes[row]);
        write_Max7219(1, row + 1, flatMouth1[row]);
        write_Max7219(2, row + 1, flatMouth2[row]);
        write_Max7219(3, row + 1, normalEyes[row]);
    }
}

void faceControl::squintface() {
    for(uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
        write_Max7219(0, row + 1, squintEyesLeft[row]);
        write_Max7219(1, row + 1, smileMouth1[row]);
        write_Max7219(2, row + 1, smileMouth2[row]);
        write_Max7219(3, row + 1, squintEyesRight[row]);
    }
}

void faceControl::surprisedface() {
    for(uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
        write_Max7219(0, row + 1, surprisedEyes[row]);
        write_Max7219(1, row + 1, openMouth1[row]);
        write_Max7219(2, row + 1, openMouth2[row]);
        write_Max7219(3, row + 1, surprisedEyes[row]);
    }
}

void faceControl::winkface() {
  isWinking = true;
}

void faceControl::sadface() {
    for (uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
        write_Max7219(0, row + 1, sadEyes[row]);
        write_Max7219(1, row + 1, sadMouth1[row]);
        write_Max7219(2, row + 1, sadMouth2[row]);
        write_Max7219(3, row + 1, sadEyes[row]);
    }
}

void faceControl::angryface() {
    for(uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
        write_Max7219(0, row + 1, angryEyesLeft[row]);
        write_Max7219(1, row + 1, flatMouth1[row]);
        write_Max7219(2, row + 1, flatMouth2[row]);
        write_Max7219(3, row + 1, angryEyesRight[row]);
    }
}

void faceControl::winking() {
    unsigned long currentMillis = millis();
    if (isWinking && currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        winkState = !winkState;
        if (winkState) {
            // 눈을 뜬 상태
            for(uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
                write_Max7219(0, row + 1, winkRight[row]);
                write_Max7219(1, row + 1, winkMouth1[row]);
                write_Max7219(2, row + 1, winkMouth2[row]);
                write_Max7219(3, row + 1, winkRight[row]);
            }
        } else {
            // 눈을 깜빡이는 상태
            for(uint8_t row = 0; row < NUMBER_OF_ROWS; row++) {
                write_Max7219(0, row + 1, winkLeft[row]);
                write_Max7219(1, row + 1, winkMouth1[row]);
                write_Max7219(2, row + 1, winkMouth2[row]);
                write_Max7219(3, row + 1, winkRight[row]);
            }
        }
    }
}
