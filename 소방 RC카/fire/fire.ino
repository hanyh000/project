#include <SoftwareSerial.h>

SoftwareSerial bluetooth(0, 1);

#define IN1 3
#define IN2 4
#define IN3 5
#define IN4 6
#define ENA 2
#define ENB 7

#define trigPin1 9
#define echoPin1 8
#define trigPin2 12
#define echoPin2 11

#define ON 10
#define ON2 13

const int flameSensorPin = A2;

long duration, sensor1val, sensor2val;
int distance;
char serialData;

bool isFlameDetected = false;
bool isUltrasonicActive = true;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT); 
  
  pinMode(flameSensorPin, INPUT);
  pinMode(ON, OUTPUT);
  pinMode(ON2, OUTPUT);

  Serial.begin(9600);
  bluetooth.begin(9600);
}

void setMotorSpeed(unsigned char speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void moveRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotor() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void driveSensor(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.4;
}

void loop() {
  // 블루투스 명령 처리
  if (bluetooth.available() > 0) {
    serialData = bluetooth.read();
    setMotorSpeed(255);

    switch (serialData) {
      case 'w':
        moveForward();
        delay(300);
        stopMotor();
        break;
        
      case 'a':
        moveLeft();
        delay(300);
        stopMotor();
        break;
        
      case 's':
        moveBackward();
        delay(300);
        stopMotor();
        break;
        
      case 'd':
        moveRight();
        delay(300);
        stopMotor();
        break;

      case 'x':
        stopMotor();
        break;
        
      case 'p': // 펌프 ON
        if (!isFlameDetected) {
          isUltrasonicActive = false;
          digitalWrite(ON, HIGH);
          digitalWrite(ON2, HIGH);
          isFlameDetected = true;
        }
        break;

      case 'o': // 펌프 OFF
        digitalWrite(ON, LOW);
        digitalWrite(ON2, LOW);
        isFlameDetected = false;
        isUltrasonicActive = true;
        break;
    }
  }

  // 초음파 센서 장애물 감지
  if (isUltrasonicActive) {
    driveSensor(trigPin1, echoPin1);
    sensor1val = distance;

    driveSensor(trigPin2, echoPin2);
    sensor2val = distance;

    if (sensor1val < 15 || sensor2val < 15) {
      stopMotor();
    }
  }

  // 불꽃 자동 감지
  int flameValue = analogRead(flameSensorPin);
  if (flameValue < 500 && !isFlameDetected) {
    stopMotor();
    isUltrasonicActive = false;
    digitalWrite(ON, HIGH);
    digitalWrite(ON2, HIGH);
    isFlameDetected = true;
  }
}