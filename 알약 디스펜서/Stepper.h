#ifndef STEPPER_H_
#define STEPPER_H_
 
#include <avr/io.h>
#include <util/delay.h>
 
#define STEPPER_PORT PORTC
#define STEPPER_DDR  DDRC
 
#define STEPPER_60STEP  10   // 60° 회전 (6칸 약통 기준 한 칸)
 
static const uint8_t step_seq[4] = {0x09, 0x03, 0x06, 0x0C};
 
static inline void STEPPER_Init(void) {
    STEPPER_DDR = 0x0F;  // PC0~PC3 출력
}
 
static inline void STEPPER_Rotate(uint8_t steps, uint8_t direction, uint16_t delay_ms) {
    for (uint8_t i = 0; i < steps; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            if (direction)
                STEPPER_PORT = step_seq[j];
            else
                STEPPER_PORT = step_seq[3 - j];
            _delay_ms(delay_ms);
        }
    }
    STEPPER_PORT = 0x00; // 정지
}
 
#endif
 
