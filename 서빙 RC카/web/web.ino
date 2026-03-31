#include <ESP8266WiFi.h>

const char* ssid = "HJ1234";           // 핫스팟 SSID
const char* password = "00000000";      // 핫스팟 비밀번호
const char* serverAddress = "182.209.25.90";    // 집 컴퓨터의 외부 IP 주소
const int serverPort = 80;                      // 집 컴퓨터의 웹 서버 포트

const int TRIG_PIN = D5; // 초음파 센서의 TRIG 핀
const int ECHO_PIN = D6; // 초음파 센서의 ECHO 핀
const int SIGNAL_PIN = D7; // 거리가 20cm 이하일 때 송신할 핀

WiFiClient client;

void setup() {
    Serial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(SIGNAL_PIN, OUTPUT); // D7 핀을 출력으로 설정

    // Connect to WiFi network
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
}

void loop() {
    long duration, distance;

    // Trigger ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);  
    delayMicroseconds(2); 
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10); 
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure the duration of the echo
    duration = pulseIn(ECHO_PIN, HIGH);
    
    // Calculate distance in cm
    distance = duration * 0.034 / 2;

    // Convert distance to string
    String distanceStr = String(distance);

    // Send HTTP request to the server
    if (client.connect(serverAddress, serverPort)) {
        client.print("GET /update?value=");
        client.print(distanceStr);
        client.println(" HTTP/1.1");
        client.print("Host: ");
        client.println(serverAddress);
        client.println("Connection: close");
        client.println();
        client.flush(); // 버퍼를 비워서 모든 데이터를 서버로 전송
        client.stop();
        Serial.println("Data sent to server");
    } else {
        Serial.println("Connection failed");
    }

    // 7번 핀으로의 출력 처리
    if (distance <= 25) {
        digitalWrite(SIGNAL_PIN, HIGH); // If yes, set D7 pin to HIGH
    } else {
        digitalWrite(SIGNAL_PIN, LOW); // If no, set D7 pin to LOW
    }

    delay(100); // 0.1초마다 데이터 전송
}
