# 위험 감지기 (Hazard Detection Device)

> **"이동 약자와 특수장비 운전자를 위한 3단계 스마트 안전 보조 장치"**
> 초음파 센서로 거리를 측정하여 LED·부저·LCD 및 PyQt5 GUI를 통해 단계별 위험 경고를 제공하고, 충돌 방지를 위한 자동 정지 기능을 지원합니다.

![Project](https://img.shields.io/badge/Project-Embedded%20System-red)
![Platform](https://img.shields.io/badge/Platform-Arduino%20|%20PyQt5-blue)
![Tech](https://img.shields.io/badge/Language-C++%20|%20Python-orange)


---


## 1. 프로젝트 개요

* **배경**: 휠체어 이용자 및 지게차 운전자의 사각지대로 인한 교통사고 위험(73.8%)을 해소하기 위한 보조 장치 필요.
* **목적**: 실시간 거리 모니터링, 단계별 시청각 경고, 위험 시 자동 정지(인터락)를 통한 사고 예방.
* **기간**: 2025.10.22 ~ 2025.11.03 (2주)
* **역할**: 하드웨어 회로 설계, 마스터/슬레이브 펌웨어 구현, PyQt5 기반 GUI 개발.


---


## 2. 주요 기능 및 구성

### **🚦 3단계 거리 경고 시스템**
| 구간 | 거리 | LED | 부저 패턴 | GUI 상태 |
| :--- | :--- | :--- | :--- | :--- |
| **안전** | 30cm 초과 | 초록 | 없음 | 🟢 안전 |
| **주의** | 11~20cm | 노랑 | 0.5초 간격 비프음 | 🟡 주의 |
| **위험** | 1~10cm | 빨강 | 고속 비프음 / 연속음 | 🔴 위험 |

### **시스템 아키텍처 (I2C & Serial)**
* **PyQt5 GUI**: 실시간 거리 수치 표시 및 모터/부저 원격 제어 (Serial 통신)
* **Main Arduino (Master)**: 센서 측정, LCD 출력, GUI 데이터 송수신 및 I2C 명령 하달
* **Motor Arduino (Slave)**: I2C 신호에 따른 DC 모터 PWM 제어 및 주행 수행


---


## 3. 기술 스택

* **Firmware**: `C/C++` (Arduino IDE, Wire Library)
* **GUI**: `Python 3.9` (PyQt5, PySerial)
* **Communication**: `I2C` (Internal), `Serial UART` (External)
* **Hardware**: Arduino(x3), HC-SR04, L298N, LCD I2C, Piezo Buzzer


---


## 4. 주요 코드 구현 (Main Master 로직)

초음파 데이터를 가공하여 경고 단계를 결정하고, GUI로 프로토콜화된 데이터를 전송하는 핵심 루틴입니다.

```cpp
void loop() {
    long distance = measureDistance(); // 초음파 측정

    // 1. 거리별 경고 단계 결정 및 시각/청각 출력
    if (distance <= 10) {
        updateAlert("R", 2000); // 위험: 빨강 LED + 고속 부저
        stopMotors();           // 안전 인터락: 자동 정지
    } else if (distance <= 20) {
        updateAlert("Y", 1500); // 주의: 노랑 LED + 간헐 부저
    } else {
        updateAlert("G", 0);    // 안전: 초록 LED
    }

    // 2. GUI 전송 프로토콜 (접두사 기반)
    Serial.print("DISTANCE:"); Serial.println(distance);
    Serial.print("ALERT:");    Serial.println(currentLevel);
    
    delay(50); // 통신 안정화를 위한 최소 딜레이
}

```


---


## 5. 프로젝트 구조

```text
hazard-detection
 ┣ main.ino       # Main Arduino 펌웨어 (마스터: 센서·LED·부저·LCD·시리얼·I2C)
 ┣ motor.ino      # Motor Arduino 펌웨어 (슬레이브: DC 모터 제어)
 ┣ gui.py         # PyQt5 GUI (실시간 거리 표시 + 원격 제어)
 ┣ g.png          # 신호등 이미지 — 안전 (초록)
 ┣ y.png          # 신호등 이미지 — 주의 (노랑)
 ┗ r.png          # 신호등 이미지 — 위험 (빨강)


```


---


## 6. 기여도 및 결과

* **기여도**: 하드웨어 설계(100%), 펌웨어 코딩(80%), GUI 개발(100%)


* **주요 성과**:
    * **초음파 센서 기반 3단계 거리 경고 시스템** 구현 완료
    * **PyQt5 GUI** 실시간 거리 표시 및 신호등 이미지 연동 완료
    * **I2C 통신** 기반 마스터·슬레이브 모터 제어 구조 구축
    * **위험 구간 자동 정지** 안전 인터락(Safety Interlock) 로직 검증


---


## 7. 결과 및 시연

* **전체 동작 시연 영상**: [YouTube 바로가기](링크를_입력하세요)
* **결과**: 위험 감지기의 안정성과 GUI코딩 능력 향상
* 


---
