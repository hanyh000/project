#include <Wire.h>

#define A_1A 6                               // 모터드라이브 A_1A 6번핀 설정
#define A_1B 5                               // 모터드라이브 A_1B 5번핀 설정

char x = 0;
unsigned long lastMotorMs = 0;
bool motorRunning = false;

void setup() {
  Wire.begin(0);                             // 슬레이브 0번 설정시 모터 R L 설정을 변경
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);

  pinMode(A_1A, OUTPUT);                     // 출력 핀모드 A_1A
  pinMode(A_1B, OUTPUT);                     // 출력 핀모드 A_1B
}

void loop() {
  // 모터드라이버와 우노보드 연결시 PWM핀으로 설정, 0~255까지 원하는대로 모터 속도 조절이 가능
  unsigned long now = millis();

  if (x != 0) {
    if (x == 'w' || x == 'W') {
      analogWrite(A_1A, 200);
      analogWrite(A_1B, 0);                  // 전진
    } else if (x == 's' || x == 'S') {
      analogWrite(A_1A, 0);
      analogWrite(A_1B, 0);                  // 정지
    } else if (x == 'b' || x == 'B') {
      analogWrite(A_1A, 0);
      analogWrite(A_1B, 200);                // 후진
    } else if (x == 'r' || x == 'R') {
      analogWrite(A_1A, 250);
      analogWrite(A_1B, 0);                  // 우회전
    } else if (x == 'l' || x == 'L') {
      analogWrite(A_1A, 0);
      analogWrite(A_1B, 250);                // 좌회전
    } else if (x == 'y' || x == 'Y') {
      analogWrite(A_1A, 125);
      analogWrite(A_1B, 0);                  // 저속
    } else {
      analogWrite(A_1A, 0);
      analogWrite(A_1B, 0);                  // 정지
    }

    lastMotorMs = now;
    motorRunning = true;
    x = 0;
  }

  // delay(500) 대체
  if (motorRunning && now - lastMotorMs >= 500) {
    analogWrite(A_1A, 0);
    analogWrite(A_1B, 0);
    motorRunning = false;
  }
}

void receiveEvent(int bytes) {
  while (Wire.available()) {
    x = Wire.read();
  }
  Serial.println(x);
}