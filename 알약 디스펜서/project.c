#ifndef F_CPU
#define F_CPU 16000000L
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "Stepper.h"
#include "uart.h"
#include "clcd.h"

/* ─── 핀 / 상수 정의 ─── */
#define TRIG           PB0      // Trigger 신호 (출력 = PB0)
#define ECHO           PB1      // Echo    신호 (입력 = PB1)
#define BUZZER         PE4      // Buzzer  신호 (출력 = PE4)
#define SOUND_VELOCITY 340UL    // 소리 속도 (m/sec)
#define MAX_ECHO_COUNT 3        // 최대 Echo 감지 횟수

#define SERVO_PIN PD5

/* ─── 전역 상태 변수 ─── */
int state      = 0;   // 동작 상태 (0: 명령 대기, 1: 초음파 감지)
int echo_count = 0;   // Echo 감지 횟수

/* ???????????????????????????????????????
   서보 초기화  (Timer0 ? Fast PWM, 8분주)
   ??????????????????????????????????????? */
void servo_init(void) {
    TCCR0  = (1 << WGM01) | (1 << WGM00) | (1 << CS01);
    DDRD  |= (1 << SERVO_PIN);
}

/* ???????????????????????????????????????
   초음파 센서 초기화
   ??????????????????????????????????????? */
void ultrasonic_init(void) {
    DDRB |=  (1 << TRIG);
    DDRB &= ~(1 << ECHO);
}

/* ???????????????????????????????????????
   초음파 거리 측정  (반환값: mm 단위)
   ??????????????????????????????????????? */
uint16_t measure_distance(void) {
    uint32_t count = 0;

    PORTB &= ~(1 << TRIG);
    _delay_us(2);
    PORTB |=  (1 << TRIG);
    _delay_us(10);
    PORTB &= ~(1 << TRIG);

    while (!(PINB & (1 << ECHO)));
    while (  PINB & (1 << ECHO)) {
        _delay_us(1);
        count++;
        if (count > 60000) break;
    }

    return (uint16_t)((count * SOUND_VELOCITY) / (2UL * 10000UL));
}

/* ???????????????????????????????????????
   메인
   ??????????????????????????????????????? */
int main(void) {
    int i;

    USART_Init(9600);
    STEPPER_Init();
    servo_init();
    ultrasonic_init();
    DDRE |= (1 << BUZZER);

    i2c_lcd_init();
    i2c_lcd_write_string("BT Pill System");
    USART_TransmitString("System Ready\r\n");

    while (1) {

        /* ── state 0 : UART 명령 대기 ─────────────────────── */
        if (state == 0) {

            if (USART_Receive_Ready()) {
                char rx = USART_Receive();

                if (rx == 'S') {

                    /* Timer1 ? 서보 모터용 PWM 설정 (PB5) */
                    DDRB  |= 0x20;
                    TCCR1A = 0x82;
                    TCCR1B = 0x1A;
                    OCR1A  = 3000;          // 초기 위치 (0°)
                    ICR1   = 19999;
                    PORTB |= (1 << PORTB5);

                    USART_TransmitString("Pill");
                    i2c_lcd_write_string("Working.");

                    /* 스텝모터 동작 ? 60° 회전 (6칸 약통 한 칸 이동) */
                    STEPPER_Rotate(STEPPER_60STEP, 1, 170);
                    i2c_lcd_write_string("Working..");
                    _delay_ms(500);
                    i2c_lcd_write_string("Working...");

                    /* 서보모터 동작 */
                    OCR1A = 500;            // 90도 회전 (약 배출)
                    _delay_ms(500);
                    i2c_lcd_write_string("take out a pill!");
                    OCR1A = 3000;           // 0도 복구
                    _delay_ms(100);
                    PORTB &= ~(1 << PORTB5);
                    OCR1A  = 0;

                    state = 1;
                }
            }

        /* ── state 1 : 초음파 센서로 약 감지 ──────────────── */
        } else if (state == 1) {

            unsigned int distance = measure_distance();

            if (distance < 300) {

                for (i = 0; i < 30; i++) {
                    PORTE |=  (1 << BUZZER);
                    _delay_ms(1);
                    PORTE &= ~(1 << BUZZER);
                    _delay_ms(1);
                }
                _delay_ms(100);

                echo_count++;

                if (echo_count >= MAX_ECHO_COUNT) {
                    echo_count = 0;
                    state      = 0;
                }
            }
        }
    }

    return 0;
}
