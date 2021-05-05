
#include "stdint.h"
#include "avr/io.h"
#include "avr/interrupt.h"

#define RX_RING_BUFFER_SIZE 128
#define TX_RING_BUFFER_SIZE 104

#define BAUD_RATE 115200
#define SERIAL_NO_DATA 0xff

void uart_init();

uint8_t uart_read();

void uart_write(uint8_t data);

void printString(const char *s);
