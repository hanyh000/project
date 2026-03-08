#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3);

int echoPin = 8;
int trigPin = 9;
int ledPin=5,speakerPin=4;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600); 

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin,OUTPUT);
}
void loop() {
  if (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }
  // 라즈베리파이로부터 데이터 받기
  if (Serial.available()) {
    BTSerial.println(Serial.readString());
  }
  delay(100);
  digitalWrite(trigPin, LOW);
  digitalWrite(echoPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH);
  float distance = ((float)(340 * duration) / 10000) / 2;
  /* Serial.print(distance);
  Serial.println("cm"); */ 
  if(distance>30){
    digitalWrite(ledPin,LOW);
    noTone(speakerPin);
    }
  else {
    digitalWrite(ledPin,HIGH);
    tone(speakerPin,700);
    }   
  delay(1000);
}
