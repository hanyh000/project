#ifndef CLCD_H_
#define CLCD_H_

#include <avr/io.h>
#include <util/delay.h>

#define LCD_DATA_PORT PORTA
#define LCD_DATA_DDR  DDRA
#define LCD_CTRL_PORT PORTG
#define LCD_CTRL_DDR  DDRG
#define LCD_RS PG0
#define LCD_RW PG1
#define LCD_EN PG2

static inline void lcd_enable(void) {
    LCD_CTRL_PORT |= (1 << LCD_EN);
    _delay_us(1);
    LCD_CTRL_PORT &= ~(1 << LCD_EN);
    _delay_us(100);
}

static inline void lcd_send_nibble(uint8_t data) {
    LCD_DATA_PORT &= 0x0F;
    LCD_DATA_PORT |= (data & 0xF0);
    lcd_enable();
}

static inline void lcd_command(uint8_t cmd) {
    LCD_CTRL_PORT &= ~(1 << LCD_RS);
    LCD_CTRL_PORT &= ~(1 << LCD_RW);
    lcd_send_nibble(cmd);
    lcd_send_nibble(cmd << 4);
    _delay_ms(2);
}

static inline void lcd_data(uint8_t data) {
    LCD_CTRL_PORT |= (1 << LCD_RS);
    LCD_CTRL_PORT &= ~(1 << LCD_RW);
    lcd_send_nibble(data);
    lcd_send_nibble(data << 4);
    _delay_ms(2);
}

static inline void i2c_lcd_init(void) {
    LCD_DATA_DDR = 0xF0;
    LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_EN);
    _delay_ms(20);

    lcd_command(0x28); // 4비트, 2줄
    lcd_command(0x0C); // 디스플레이 ON
    lcd_command(0x06); // 자동 커서 이동
    lcd_command(0x01); // 화면 클리어
    _delay_ms(2);
}

static inline void i2c_lcd_clear(void) {
    lcd_command(0x01);
    _delay_ms(2);
}

static inline void i2c_lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_command(pos);
}

static inline void i2c_lcd_write_string(const char *str) {
    while (*str) lcd_data(*str++);
}

#endif
