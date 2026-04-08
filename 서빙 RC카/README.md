# 서빙 RC카 (Serving RC Car)

> **"오배송 제로를 향한 스마트 매장 자동 주행 서빙 로봇"**
> 앱으로 테이블을 선택하면 지정된 경로로 자동 주행하며, 웹 페이지를 통해 실시간으로 로봇의 상태를 모니터링하는 통합 시스템입니다.

![Category](https://img.shields.io/badge/Project-Embedded%20&%20Web-green)
![Platform](https://img.shields.io/badge/Platform-Arduino%20|%20Node.js-blue)
![Tech](https://img.shields.io/badge/Language-C++%20|%20JavaScript%20|%20Java-orange)

---

## 1. 프로젝트 개요

* **배경**: 신규 직원의 테이블 번호 혼동으로 인한 오배달 및 서비스 질 저하 문제를 해결하기 위해 기획.
* **목적**: 지정 테이블 자동 이동을 통한 실수를 방지하고, 관리자가 웹에서 실시간으로 로봇 상태를 확인할 수 있는 환경 구축.
* **기간**: 2024.05 ~ 2024.06 (약 1개월)
* **역할**: 하드웨어 설계 및 제작, Arduino/ESP8266 펌웨어 구현, Android 제어 앱 단독 개발.

---

## 2. 핵심 기능

| 기능 | 상세 설명 |
| :--- | :--- |
| **자동 주행 시스템** | 앱에서 테이블(1~4번) 선택 시 해당 경로로 사전 프로그래밍된 5가지 모드 조합 주행 |
| **웹 실시간 모니터링** | ESP8266이 초음파 데이터를 0.1초 주기로 전송, Node.js 서버에서 실시간 거리 수치 표시 |
| **지능형 장애물 회피** | 주행 중 초음파 센서가 25cm 이내 장애물 감지 시 즉시 정지 신호 발생 및 주행 차단 |
| **수동 조작 모드** | 블루투스 연동 앱을 통해 전/후/좌/우 직접 제어 및 RGB LED 상태 확인 기능 |

---

## 3. 시스템 아키텍처

### **Data Flow**

1.  **입력**: Android 앱 (Bluetooth) → **Arduino Uno** (메인 제어)
2.  **센서 데이터**: 초음파 센서 → **ESP8266** (WiFi 통신)
3.  **출력/모니터링**: 
    * Arduino → DC 모터 (주행)
    * ESP8266 → **Node.js 서버** → 웹 브라우저 (실시간 대시보드)

---

## 4. 기술 스택

* **Firmware**: `C/C++` (Arduino IDE)
* **Backend**: `Node.js`, `Express.js` (실시간 센서 데이터 스트리밍)
* **Mobile**: `Java` (Android Studio)
* **Communication**: `Bluetooth (HC-05)`, `WiFi (ESP8266)`, `HTTP/GET`
* **Hardware**: Arduino Uno, ESP8266, DC 모터(x2), 초음파 센서, RGB LED 링

---

## 5. 프로젝트 구조

```text
project/
├── serving.ino       # Arduino 주행 제어 (BT 수신 및 모터 드라이빙)
├── web.ino           # ESP8266 펌웨어 (센서 데이터 -> 웹 서버 REST API 전송)
└── server.js         # Node.js 웹 서버 (Express 기반 데이터 수신 및 렌더링)


---


## 6. 주요 코드 구현 (ESP8266 서버 전송)

ESP8266을 통해 초음파 센서로 측정한 거리 데이터를 특정 서버 IP의 Node.js 서버로 전송하는 핵심 로직입니다.

```cpp
void sendSensorData(int distance) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String url = "[http://192.168.0.10/update?distance=](http://192.168.0.10/update?distance=)" + String(distance);
        
        http.begin(url); // 서버 접속
        int httpCode = http.GET(); // GET 방식으로 데이터 전송
        
        if (httpCode > 0) {
            Serial.printf("[HTTP] GET... code: %d\n", httpCode);
        }
        http.end();
    }
}
```

---


## 7. 주요 기술적 해결 (Troubleshooting)

### **a. 환경에 따른 주행 정밀도 오차 보정**
* **문제**: 바닥 재질이나 배터리 잔량에 따라 `delay` 기반 주행 거리와 회전 각도가 달라지는 문제 발생.
* **해결**: 실제 구동 환경에서 테이블별 반복 주행 테스트를 실시하여 최적의 **타이밍 캘리브레이션 값**을 도출하고 코드에 반영하여 신뢰도 향상.

### **b. 통신 병목 현상 및 제어 충돌 해결**
* **문제**: ESP8266의 데이터 전송 주기와 Arduino의 모터 제어 신호가 겹치며 동작이 끊기는 현상 발생.
* **해결**: 데이터 전송 주기(0.1초)와 주행 로직의 인터럽트 우선순위를 조정하고, 통신 순서를 명확히 분리하여 부드러운 주행과 실시간 모니터링을 동시에 달성.

### **c. 하드웨어 UX 개선 (눈부심 방지)**
* **문제**: RGB LED의 고휘도 빛이 사용자 시야를 방해함.
* **해결**: 기구 설계 단계에서 **지붕형 하우징**을 제작하여 LED를 가림으로써 시각적 식별 기능은 유지하고 눈부심은 차단하는 최적의 배치를 설계함.

---

## 8. 기여도 및 결과

* **기여도**: 하드웨어 설계(70%), 펌웨어 코딩(60%), 앱 제작(100%)
* **성과**: 
    * 테이블 1~4번 자동 주행 시뮬레이션 성공
    * WiFi 기반 실시간 모니터링 시스템 구축 (1초 단위 갱신)
    * 실습 매장 환경에서의 실제 주행 시연 완료

---

## 9. 결과 및 시연

* **시연 영상**: [YouTube 바로가기](링크를_입력하세요)
* **결과**: 도메인 이용법과 웹통신 같은 익숙하지 않은 방식으로 센서값을 출력하는 새로운 경험이었다

---
