int FlexPin = A0;
// 모터 제어를 위한 핀 설정
const int enA = 9;   // ENA 핀
const int in1 = 2;   // IN1 핀
const int in2 = 3;   // IN2 핀

const int enB = 10;   // ENB 핀
const int in3 = 4;   // IN3 핀
const int in4 = 5;   // IN4 핀





int A;

bool motorRunning = false;  // 모터 동작 여부를 나타내는 변수
unsigned long lastFlexChangeTime = 0;  // 센서 값이 마지막으로 변한 시간을 기록하는 변수

void setup() {
  Serial.begin(9600);
  // 모터 제어 핀을 출력으로 설정
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // 초기에 모터를 정지 상태로 설정
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  int FlexVal;
  FlexVal = analogRead(FlexPin);
  
  // 센서 값을 시리얼 모니터에 출력
  Serial.print("sensor : ");
  Serial.println(FlexVal);

  delay(300);

  // 센서 값이 변할 때만 동작
  if (FlexVal <= 10 && !motorRunning == A) {
    // 센서 값이 0이고 모터가 작동 중이지 않을 때만 실행
    motorRunning = true;  // 모터 작동 상태로 변경
    
    // 센서 값이 0일 때 모터를 전진 방향으로 회전
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // 모터 속도 설정 (PWM 사용)
    analogWrite(enA, 255);  // 최대 속도 설정
    analogWrite(enB, 255);

    delay(300);  // 0.3초 동안 전진
    
    // 모터를 정지 상태로 변경
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(1000);  // 1초 동안 정지
    
    motorRunning = false;  // 모터 정지 상태로 변경
    A = 0;
  }
  
  // 센서 값이 변할 때만 동작
  if (FlexVal > 10 && !motorRunning == !A) {
    // 센서 값이 10을 초과하고 모터가 작동 중이지 않을 때만 실행
    motorRunning = true;  // 모터 작동 상태로 변경
    
    // 모터를 후진 방향으로 회전
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    
    // 모터 속도 설정 (PWM 사용)
    analogWrite(enA, 255);  // 최대 속도 설정
    analogWrite(enB, 255);

    delay(300);  // 0.3초 동안 후진
    
    // 모터를 정지 상태로 변경
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    
    delay(1000);  // 1초 동안 정지
    
    motorRunning = false;  // 모터 정지 상태로 변경
    A = 1;
  }
}
