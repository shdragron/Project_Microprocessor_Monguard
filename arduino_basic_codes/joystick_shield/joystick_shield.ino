const int buttonPin2 = 2;                           // 조이스틱 쉴드의 버튼이 누르는 걸 입력받기 위해 선언
const int buttonPin3 = 3;
const int buttonPin4 = 4;
const int buttonPin5 = 5;
 
void setup() {
 
  Serial.begin(9600);                               // 시리얼 통신을 시작하며, 통신속도는 9600
 
  pinMode(buttonPin2, INPUT_PULLUP );
  pinMode(buttonPin3, INPUT_PULLUP );
  pinMode(buttonPin4, INPUT_PULLUP );
  pinMode(buttonPin5, INPUT_PULLUP );
}
 
void loop() {
 
 
  int X = analogRead(0);                           // 변수 X에 아날로그 0번핀에 입력되는 신호를 대입
  int Y = analogRead(1);                           // 변수 Y에 아날로그 1번핀에 입력되는 신호를 대입
 
  int buttonValue2 = digitalRead(2);               // buttonValue값 선언
  int buttonValue3 = digitalRead(3);
  int buttonValue4 = digitalRead(4);
  int buttonValue5 = digitalRead(5);
 
 
  Serial.print("joy stick  ");                       // 조이스틱 x값, y값 시리얼모니터에 출력
  Serial.print("X");
  Serial.print(":");
  Serial.print(X);
  Serial.print("  ");
  Serial.print("Y");
  Serial.print(":");
  Serial.println(Y);
 
  if (buttonValue2 == LOW) {                       // if문을 이용하여 각 버튼이 눌리면 알파벳이 시리얼모니터에 출력되도록 설정
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("A: Yes  B: No  C: No  D: No");
  }
  if (buttonValue3 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("A: No  B: Yes  C: No  D: No");
  }
  if (buttonValue4 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("A: No  B: No  C: Yes  D: No");
  }
  if (buttonValue5 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("A: No  B: No  C: No  D: Yes");
  }
 
  delay(50);                                        // 0.5초동안 지속
}