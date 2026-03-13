#include <Wire.h>

#define A_1A 6                               // 모터드라이브 A_1A 6번핀 설정
#define A_1B 5                               // 모터드라이브 A_1B 5번핀 설정     

char x = 0;

void setup() {
  Wire.begin(0);     //슬레이브 0번 설정시 모터 R L 설정을 변경
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);

  pinMode (A_1A, OUTPUT);           // 출력 핀모드 A_1A
  pinMode (A_1B, OUTPUT);           // 출력 핀모드 A_1B
}


void loop() {

  // 모터드라이버와 우노보드 연결시 PWM핀으로 설정,  0~255까지 원하는대로 모터 속도 조절이 가능 
  if(x == 'w' || x == 'W'){
  analogWrite(A_1A, 200);                   
  analogWrite(A_1B, 0);                    //w수신 시 3초 전진
  delay (500);
  }else if(x == 's' || x == 'S'){
    analogWrite(A_1A, 0);                     
    analogWrite(A_1B, 0);                  //s수신 시 3초 정지
    delay (500);
  }else if(x == 'b'|| x == 'B'){
    analogWrite(A_1A, 0);                     
    analogWrite(A_1B, 200);                //b수신 시 3초 후진
    delay (500);
  }else if(x == 'r'|| x == 'R'){
    analogWrite(A_1A, 250);                   
    analogWrite(A_1B, 0);                  //r수신 시 3초 우회전
    delay (500);
  }else if(x == 'l'|| x == 'L'){
    analogWrite(A_1A, 0);                     
    analogWrite(A_1B, 250);                 //l수신 시 3초 좌회전
    delay (500);
  }else if(x == 'y'|| x == 'Y'){
    analogWrite(A_1A, 125);                   
    analogWrite(A_1B, 0);                  //r수신 시 3초 우회전
    delay (500);
  }else{
    analogWrite(A_1A, 0);                 
    analogWrite(A_1B, 0);                //모두 아닐 시 정지     
  }
}
void receiveEvent(int bytes) {
  while (Wire.available()) {
    x = Wire.read();
  }
  Serial.println(x); 
}