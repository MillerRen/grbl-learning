#include "serial.h"

// grbl串口操作案例
// 声明：本实例由任羽飞编写，禁止抄袭，转载请说明出处

uint8_t serial_tx_buffer[TX_RING_BUFFER_SIZE+1];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;

uint8_t serial_rx_buffer[RX_RING_BUFFER_SIZE+1];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

// 初始化串口波特率为115200，8位无奇偶校验1停止位，开启接收，发送功能和接收完成中断
void serial_init () {
    uint16_t baud_prescaler = F_CPU / (8L*BAUD_RATE) - 1;
    UCSR0A |= (1<<U2X0);
    
    UBRR0H = baud_prescaler >> 8;
    UBRR0L = baud_prescaler;

    UCSR0B |= (1<<RXEN0|1<<TXEN0|1<<RXCIE0);
}

// 写入发送环形队列
void serial_write (uint8_t data) {
    uint8_t next_head = serial_tx_buffer_head + 1;
    if(next_head == TX_RING_BUFFER_SIZE) {
        next_head = 0;
    }
    while (next_head == serial_tx_buffer_tail)
    {
        
    }

    serial_tx_buffer[serial_tx_buffer_head] = data;
    serial_tx_buffer_head = next_head;
    
    UCSR0B |= (1<<UDRIE0);
}

// 从接收环形队列读取数据 
uint8_t serial_read () {
    uint8_t tail = serial_rx_buffer_tail;
    if (serial_rx_buffer_head == tail) {
        return SERIAL_NO_DATA;
    }
    
    uint8_t data = serial_rx_buffer[tail];
    
    tail++;
    if(tail == RX_RING_BUFFER_SIZE) {
        tail = 0;
    }
    serial_rx_buffer_tail = tail;

    return data;
}

void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}

void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}

// 串口接收数据完成，把数据保存到接收环形队列
ISR(USART_RX_vect) {
    uint8_t data = UDR0;
    uint8_t next_head = serial_rx_buffer_head + 1;
    if (next_head == RX_RING_BUFFER_SIZE) {
        next_head = 0;
    }
    if (next_head != serial_rx_buffer_tail)
    {
        serial_rx_buffer[serial_rx_buffer_head] = data;
        serial_rx_buffer_head = next_head;
    }    
}

// 串口数据寄存器为空时，从发送环形队列得到数据并发送到串口
ISR(USART_UDRE_vect) {
    uint8_t tail = serial_tx_buffer_tail;
    UDR0 = serial_tx_buffer[tail];
    tail++;
    if (tail == TX_RING_BUFFER_SIZE) {
        tail = 0;
    }
    serial_tx_buffer_tail = tail;

    if (tail == serial_tx_buffer_head) {
        UCSR0B &= ~(1<<UDRIE0);
    }
}


