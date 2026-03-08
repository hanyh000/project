#include <DHT.h>

#define DHTPIN A0
#define DHTTYPE DHT11

const int motorPin = 9;

boolean motorManualMode = false;
boolean motorOn = false;

DHT dht(DHTPIN, DHTTYPE);

unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 1000;  // 1초마다 센서 읽기

void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, HIGH);  // 초기 모터 OFF
  dht.begin();
}

void loop() {
  // ── 시리얼 명령 수신 (딜레이 없이 항상 체크) ──────────────
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '3') {
      motorManualMode = true;
      Serial.println("수동모드 활성화");

    } else if (command == '4') {
      motorManualMode = false;
      Serial.println("자동모드 활성화");

    } else if (command == '7') {
      if (motorManualMode) {
        motorOn = true;
        digitalWrite(motorPin, LOW);
        Serial.println("DC 모터 켜짐");
      }

    } else if (command == '8') {
      if (motorManualMode) {
        motorOn = false;
        digitalWrite(motorPin, HIGH);
        Serial.println("DC 모터 꺼짐");
      }
    }
  }

  // ── 1초마다 센서 읽기 + 자동모드 처리 (millis 비블로킹) ───
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      Serial.print("tem,");
      Serial.println(t);
      Serial.print("hum,");
      Serial.println(h);

      // 자동모드일 때만 온도로 모터 제어
      if (!motorManualMode) {
        if (t >= 24) {
          digitalWrite(motorPin, LOW);
        } else {
          digitalWrite(motorPin, HIGH);
        }
      }
    }
  }
}