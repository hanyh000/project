#ifndef UART_H_
#define UART_H_

#include <avr/io.h>

static inline void USART_Init(unsigned int baud) {
    unsigned int ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static inline void USART_Transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

static inline void USART_TransmitString(const char *str) {
    while (*str) USART_Transmit(*str++);
}

static inline uint8_t USART_Receive_Ready(void) {
    return (UCSR0A & (1 << RXC0));
}

static inline unsigned char USART_Receive(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

#endif
