#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

char data = 0;
// === 핀 설정 ===
int trigPin1 = 2;
int echoPin1 = 3;
int buzzer   = 10;
int rLED = 11, yLED = 12, gLED = 13;

// --- 모터 핀 (L298N 예시) ---
int enA = 9; // 모터 A 속도 (PWM)
int in1 = 8; // 모터 A 방향 1
int in2 = 7; // 모터 A 방향 2
int enB = 6; // 모터 B 속도 (PWM)
int in3 = 5; // 모터 B 방향 1
int in4 = 4; // 모터 B 방향 2
// ------------------------------

// === 전역 변수 ===
long duration1;
long distance1;

// === LED 임계 ===
const int RED_MAX    = 10;
const int YEL_MAX    = 20;
const int GRN_MAX    = 30;

// === 자동 부저 파라미터 ===
const int RED_CONT_CM         = 4;
const int RED_BEEP_MIN_INT_MS = 60;
const int RED_BEEP_MAX_INT_MS = 300;
const int MIN_ON_MS           = 40;
const int YEL_BEEP_INTERVAL_MS = 500;
const int YEL_BEEP_ON_MS       = 80;
const int YEL_BEEP_FREQ        = 1500;

static unsigned long lastToggleMs = 0;
static bool buzOn = false;

// === 시리얼 명령 수신용 변수 ===
String receivedCommand = ""; // 파이썬에서 받은 명령어
bool commandComplete = false;  // 명령어 수신 완료 플래그

void setup() 
{
  Wire.begin(0);
  Wire.begin(1);
  
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(rLED, OUTPUT);
  pinMode(yLED, OUTPUT);
  pinMode(gLED, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // --- 모터 핀 모드 설정 ---
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
  receivedCommand.reserve(20); // 메모리 미리 할당

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Distance :");
}

// --- 모터 제어 함수 ---
void goForward() { // CW
  Serial.println("MOTOR : CW"); // 파이썬으로 상태 전송
  data = 'w';
  Serial.write("DATA : ");
  Serial.write(data);
}

void goBack() 
{ 
  Serial.println("DATA : CCW");
  data = 'b';
  Serial.write(data);
}

void turnLeft() 
{ 
  Serial.println("DATA : LEFT");
  data = 'l';
  Serial.write(data);
}

void turnRight()
{ 
  Serial.println("MOTOR : RIGHT");
  data = 'r';
  Serial.write(data);
}

void stopMotors() 
{ 
  Serial.println("MOTOR : STOP");
  data = 's';
  Serial.write(data);
}
// ------------------------

void loop() 
{
  Wire.beginTransmission(0);
  Wire.write(data);             
  Wire.endTransmission(0);
  
  Wire.beginTransmission(1);
  Wire.write(data);            
  Wire.endTransmission(1);  
  
  
  // --- 1. 파이썬으로부터 시리얼 명령 수신 ---
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') { // 줄바꿈 문자(\n)를 받으면
      commandComplete = true; // 명령어 완성
    } else {
      receivedCommand += inChar;
    }
  }

  // --- 2. 완성된 명령 처리 ---
  if (commandComplete) {
    receivedCommand.trim(); // 앞뒤 공백 제거
    
    if (receivedCommand == "CW") {
      goForward();
    } else if (receivedCommand == "CCW") {
      goBack();
    } else if (receivedCommand == "LEFT") {
      turnLeft();
    } else if (receivedCommand == "RIGHT") {
      turnRight();
    } else if (receivedCommand == "STOP") {
      stopMotors();
    } else if (receivedCommand == "BUZZER") {
      // [수정] 1500Hz 소리를 0.1초(100ms)간 짧게 울림
      tone(buzzer, 1500, 100); 
      Serial.println("BUZZER_BEEP"); // 상태 전송
    }

    // 다음 명령을 받기 위해 변수 초기화
    receivedCommand = "";
    commandComplete = false;
  }

  // --- 3. 초음파 센서 측정 ---
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(100);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(100);
  digitalWrite(trigPin1, LOW);

  duration1 = pulseIn(echoPin1, HIGH, 10000UL); // 타임아웃 10ms
  if (duration1 == 0) { distance1 = -1; }
  else { distance1 = duration1 * 0.034 / 2; }

  // --- 4. 센서 값 파이썬으로 전송 ---
  Serial.print("DISTANCE : ");
  Serial.println(distance1);
  delay(50);

    // --- 4.5 LCD 출력 ---
  lcd.setCursor(0,1);
  if(distance1 > 0)
  {
    lcd.print(distance1);
    lcd.print(" cm   "); // 기존 값 지우기
  } 
  else
  {
    lcd.print("---    "); // 값 없음 표시
  }

  // --- 5. LED 단계 표시 ---
  if (distance1 > 0 && distance1 <= RED_MAX) {
    Serial.println("ALERT : R");
    stopMotors();
    digitalWrite(gLED, LOW); digitalWrite(yLED, LOW); digitalWrite(rLED, HIGH);
  } else if (distance1 <= YEL_MAX) {
    Serial.println("ALERT : Y");
    data = 'Y';
    digitalWrite(gLED, LOW); digitalWrite(yLED, HIGH); digitalWrite(rLED, LOW);
  } else if (distance1 <= GRN_MAX) {
    Serial.println("ALERT : G");
    digitalWrite(gLED, HIGH); digitalWrite(yLED, LOW); digitalWrite(rLED, LOW);
  } else {
    digitalWrite(gLED, HIGH); digitalWrite(yLED, LOW); digitalWrite(rLED, LOW);
  }

  // --- 6. 거리별 자동 부저 로직 ---
  // (수동 부저가 '삑' 하고 끝나는 동안 잠시 중첩될 수 있지만,
  //  수동 부저는 짧게 끝나므로 자동 부저 로직을 항상 실행합니다.)
  unsigned long now = millis();

  if (distance1 > 0 && distance1 <= RED_MAX) { // 빨간 구간
    if (distance1 <= RED_CONT_CM) { // 연속음
      if (!buzOn) {
        tone(buzzer, 1000);
        buzOn = true;
        Serial.println("BUZZ : CONTINUOUS");
      }
    } else { // 빨간 삑삑이
      long intervalMs = map(distance1, RED_MAX, RED_CONT_CM + 1, RED_BEEP_MAX_INT_MS, RED_BEEP_MIN_INT_MS);
      intervalMs = constrain(intervalMs, RED_BEEP_MIN_INT_MS, RED_BEEP_MAX_INT_MS);
      long onMs = max(MIN_ON_MS, (long)(intervalMs * 0.3));
      int freq = map(distance1, RED_MAX, RED_CONT_CM + 1, 1800, 3000);
      freq = constrain(freq, 600, 2000);

      if (!buzOn) {
        if (now - lastToggleMs >= (unsigned long)intervalMs) {
          tone(buzzer, freq); buzOn = true; lastToggleMs = now;
        }
      } else {
        if (now - lastToggleMs >= (unsigned long)onMs) {
          noTone(buzzer); buzOn = false; lastToggleMs = now;
        }
      }
    }
  }
  else if (distance1 > RED_MAX && distance1 <= YEL_MAX) { // 노란 구간
    long intervalMs = YEL_BEEP_INTERVAL_MS;
    long onMs = YEL_BEEP_ON_MS;
    int freq = YEL_BEEP_FREQ;

    if (!buzOn) {
      if (now - lastToggleMs >= (unsigned long)intervalMs) {
        tone(buzzer, freq); buzOn = true; lastToggleMs = now;
        Serial.println("BUZZ : YELLOW BEEP");
      }
    } else {
      if (now - lastToggleMs >= (unsigned long)onMs) {
        noTone(buzzer); buzOn = false; lastToggleMs = now;
      }
    }
  }
  else { // 그 외 (초록색, 감지X)
    noTone(buzzer);
    buzOn = false;
  }

  // --- 7. 최소 딜레이 ---
  delay(10); // 루프가 너무 빨리 도는 것 방지
}