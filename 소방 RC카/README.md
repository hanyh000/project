# 소방 RC카 (Firefighting RC Car)

> **"화재 현장의 인명 피해 최소화를 위한 자율주행 소방 로봇"**
> 불꽃 감지 센서를 통한 자동 화원 탐색 및 초기 진화, 블루투스 기반 원격 제어를 지원하는 스마트 소방 솔루션입니다.

![Award](https://img.shields.io/badge/Project-Embedded%20System-red)
![Platform](https://img.shields.io/badge/Platform-Arduino%20Uno-blue)
![Tech](https://img.shields.io/badge/Language-C++%20(Arduino)%20|%20Java%20(Android)-orange)

---


## 1. 프로젝트 개요

* **배경**: 화재 현장 투입 소방관의 부상 및 사망 위험을 줄이기 위해 원격 제어 및 자동 진화가 가능한 로봇의 필요성 대두.
* **목적**: 위험 지역 선행 탐색, 불꽃 감지를 통한 자동 소화(Water Pump), 실시간 장애물 회피 주행.
* **기간**: 2023.11 ~ 2023.12 (약 3주)
* **역할**: 하드웨어 회로 설계, 아두이노 펌웨어 구현, 안드로이드 블루투스 제어 앱 개발.


---


## 2. 핵심 기능 및 동작

| 기능 | 상세 설명 |
| :--- | :--- |
| **자동 화재 진압** | 불꽃 감지 센서 데이터가 임계값(500) 미만일 때 화재로 판단, 즉시 정지 후 워터펌프 가동 |
| **하이브리드 제어** | 자동 감지 모드와 블루투스 앱을 이용한 수동 원격 조작 모드 실시간 전환 지원 |
| **장애물 회피** | 전면/측면 초음파 센서를 활용해 15cm 이내 장애물 감지 시 자동 정지 및 충돌 방지 |
| **스마트 펌프 제어** | 화원 탐지 시에만 펌프를 구동하여 용수 및 배터리 효율 극대화 |


---


## 3. 시스템 아키텍처 및 핀 구성

### **Hardware Connection**
* **Main Controller**: Arduino Uno
* **Communication**: Bluetooth HC-06 (Serial Comm)
* **Drive System**: L298N Motor Driver + RC Tank Chassis

### **Pin Mapping**
* **Motor Driver**: IN1(3), IN2(4), IN3(5), IN4(6) / ENA(2), ENB(7)
* **Sensors**: Ultrasonic(Trig 9,12 / Echo 8,11), Flame Sensor(A2)
* **Actuator**: Water Pump(10, 13)


---


## 4. 주요 코드 구현 (자동 진화 로직)

불꽃 센서의 아날로그 값을 읽어 화재 발생 여부를 판단하고, 모터 제어와 펌프 구동을 연동하는 핵심 알고리즘입니다.

```cpp
void loop() {
    int flameValue = analogRead(FLAME_SENSOR); // 불꽃 감지값 수신

    // 1. 화재 감지 시 (임계값 500 미만)
    if (flameValue < 500) {
        stopMotors();          // 주행 정지
        digitalWrite(PUMP, HIGH); // 워터펌프 가동
        Serial.println("Fire Detected! Extinguishing...");
    } 
    // 2. 수동 제어 모드 (블루투스 명령 수신)
    else if (bluetooth.available()) {
        char cmd = bluetooth.read();
        handleManualControl(cmd);
    }
}
```


---


## 5. 기술적 해결 (Troubleshooting)

### **a. 불꽃 감지 센서의 유효 거리 한계**
* **문제**: 단일 센서의 감지 거리가 짧아 먼 거리의 화원을 자동으로 찾아가기 어려움.
* **해결**: 블루투스 앱으로 위험 지역 근처까지 수동 배치한 뒤, 화원 근접 시 자동 모드로 전환하는 **하이브리드 운용 방식**을 채택하여 탐지 성공률 보완.

### **b. 고출력 부하로 인한 전력 부족**
* **문제**: 모터와 워터펌프 동시 구동 시 전압 강하가 발생하여 아두이노가 리셋되는 현상 발생.
* **해결**: 구동부 전원을 분리하고 대용량 보조배터리와 승압 회로(Step-up Converter)를 적용하여 안정적인 전력 공급 체계 구축.


---


## 6. 기여도 및 성과

* **하드웨어 설계 (65%)**: 회로 구성 및 부하 계산을 통한 전력 안정화.
* **펌웨어 개발 (60%)**: 센서 데이터 필터링 및 블루투스 프로토콜 설계.
* **앱 제작 (100%)**: MIT App Inventor를 활용한 전용 컨트롤러 앱 UI/UX 디자인 및 로직 구현.


---


## 7. 결과 및 시연

* **시연 영상**: [YouTube 바로가기](링크를_입력하세요)
* **결과**: 수동조작하여 센서 감지시 자동으로 물을 뿌리거나 수동으로 물을 뿌려 불을 끈다.
