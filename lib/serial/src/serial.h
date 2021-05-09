
#include "stdint.h"
#include "avr/io.h"
#include "avr/interrupt.h"


// grbl串口操作案例
// 声明：本实例由任羽飞编写，禁止抄袭，转载请说明出处
#define RX_RING_BUFFER_SIZE 128
#define TX_RING_BUFFER_SIZE 104

#define BAUD_RATE 115200
#define SERIAL_NO_DATA 0xff

void serial_init();

uint8_t serial_read();

void serial_write(uint8_t data);

void serial_reset_read_buffer();

void printString(const char *s);
