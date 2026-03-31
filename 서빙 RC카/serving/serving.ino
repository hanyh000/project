#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

#define PIN 6            // LED 데이터 핀이 연결된 아두이노 핀
#define NUMPIXELS 12     // LED 링에 있는 LED 수
#define TRIG_PIN 10      // 초음파 센서 Trig 핀
#define ECHO_PIN 11      // 초음파 센서 Echo 핀
#define PAUSE_PIN 7      // 초음파 센서에서 HIGH 신호를 받는 핀

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// 모터 A를 제어하는 핀
const int motorA1 = 2; // A1A
const int motorA2 = 3; // A1B
// 모터 B를 제어하는 핀
const int motorB1 = 4; // B1A
const int motorB2 = 5; // B1B

// 블루투스 통신 핀 설정
SoftwareSerial bluetooth(8, 9); // RX, TX

unsigned long previousMillis = 0;
const long interval = 50;  // LED 업데이트 간격 (밀리초)

bool isPaused = false;

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PAUSE_PIN, INPUT);

  Serial.begin(9600); // 시리얼 통신 초기화
  bluetooth.begin(9600); // 블루투스 통신 초기화

  pixels.begin(); // 네오픽셀 라이브러리 초기화
  pixels.show();  // 모든 픽셀을 끄기
}

void loop() {
  // 현재 시간 확인
  unsigned long currentMillis = millis();

  // 초음파 센서로 거리 측정
  long distance = measureDistance();

  // 일시정지 신호 감지
  if (digitalRead(PAUSE_PIN) == HIGH) {
    stopMotors();
    isPaused = true;
  } else if (isPaused && digitalRead(PAUSE_PIN) == LOW) {
    isPaused = false;
  }

  if (!isPaused) {
    if (distance <= 20) {
      stopMotors();
    } else {
      // 모터 제어
      if (Serial.available() > 0 || bluetooth.available() > 0) {
        char command;
        if (Serial.available() > 0) {
          command = Serial.read();
        } else {
          command = bluetooth.read();
        }
        if (command == 'w') {
          forward();
        } else if (command == 's') {
          backward();
        } else if (command == 'a') {
          left();
          delay(330);
          stopMotors();
        } else if (command == 'd') {
          right();
          delay(330);
          stopMotors();
        } else if (command == 'p') {
          stopMotors();
        } else if (command >= '1' && command <= '4') {
          moveOnPath(command - '0');
        }
      }
    }
  }

  // LED 업데이트
  if (currentMillis - previousMillis >= interval) {
    rainbowChase();
    previousMillis = currentMillis;
  }
}

// 주어진 경로로 이동
void moveOnPath(int pathNumber) {
  if (pathNumber == 1) {
    executePathSegment(forward, 1200);
    executePathSegment(left, 430);
    executePathSegment(forward, 1200);
    executePathSegment(stopMotors, 1200);
    executePathSegment(left, 860);
    executePathSegment(stopMotors, 1200);
    executePathSegment(forward, 1200);
    executePathSegment(right, 360);
    executePathSegment(forward, 1200);
    executePathSegment(stopMotors, 1200);
  } else if (pathNumber == 2) {
    executePathSegment(forward, 1200);
    executePathSegment(right, 360);
    executePathSegment(forward, 1200);
    executePathSegment(stopMotors, 1200);
    executePathSegment(right, 800);
    executePathSegment(stopMotors, 1200);
    executePathSegment(forward, 1200);
    executePathSegment(left, 430);
    executePathSegment(forward, 1200);
    executePathSegment(stopMotors, 1200);
  } else if (pathNumber == 3) {
    executePathSegment(forward, 2200);
    executePathSegment(left, 445);
    executePathSegment(forward, 1200);
    executePathSegment(stopMotors, 1200);
    executePathSegment(left, 975);
    executePathSegment(stopMotors, 1200);
    executePathSegment(forward, 1200);
    executePathSegment(right, 360);
    executePathSegment(forward, 2200);
    executePathSegment(stopMotors, 1200);
  } else if (pathNumber == 4) {
    executePathSegment(forward, 2200);
    executePathSegment(right, 360);
    executePathSegment(forward, 1200);
    executePathSegment(stopMotors, 1200);
    executePathSegment(right, 800);
    executePathSegment(stopMotors, 1200);
    executePathSegment(forward, 1200);
    executePathSegment(left, 430);
    executePathSegment(forward, 2200);
    executePathSegment(stopMotors, 1200);
  }
}

// 후진 함수
void backward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

// 전진 함수
void forward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

// 우회전 함수
void right() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

// 좌회전 함수
void left() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

// 정지 함수
void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

// 경로 세그먼트를 실행하는 함수
void executePathSegment(void (*motorFunc)(), int duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    if (digitalRead(PAUSE_PIN) == HIGH) {
      stopMotors();
      while (digitalRead(PAUSE_PIN) == HIGH) {
        delay(10); // 일시정지 상태 유지
      }
      startTime = millis(); // 일시정지 후 경로 재개
    }
    motorFunc();
  }
  stopMotors();
}

// 무지개 색 변경 함수
void rainbowChase() {
  static uint8_t colorStep = 0;
  for(int i=0; i<NUMPIXELS; i++) {
    int pixelIndex = (i + colorStep) % 256;
    pixels.setPixelColor(i, Wheel(pixelIndex));
  }
  pixels.show();
  colorStep++;
}

// 무지개 색상 계산 함수
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

// 초음파 센서로 거리 측정 함수
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}
