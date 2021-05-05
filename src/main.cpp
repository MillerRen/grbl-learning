// UCSRnA and UCSRnB and UCSRnC: USART Control and Status Register n A 串口控制与状态寄存器
// UDRn: USART I/O Data Register n 串口IO数据寄存器
// UBRRnL and UBRRnH – USART Baud Rate Registers 串口波特率寄存器
// 声明：本实例由任羽飞编写，禁止抄袭，转载请说明出处
#include "stdlib.h"
#include "stdint.h"
#include "avr/io.h"
#include "avr/interrupt.h"

#define BAUD_RATE 115200

volatile uint8_t data;

int main (void) {
    uint16_t UBBRn_value = F_CPU/(8*BAUD_RATE)-1; // 波特率计算公式
    UCSR0A |= (1<<U2X0); // 开启二倍传输，误差更小
    UBRR0H = UBBRn_value >> 8; // 右移8位得到高位
    UBRR0L = UBBRn_value; // 低位直接存
    // 使能串口发送,接收，接收完成中断使能
    UCSR0B |= (1<<TXEN0|1<<RXEN0|1<<RXCIE0); 
    sei(); // 开启总中断
    for (;;)
    {
        if (data != 0) {
            UCSR0B |= (1<<UDRIE0);
        }
    }
    return 0;
}

// 串口接收中断处理
ISR(USART_RX_vect) {
    data = UDR0; // 从数据寄存器UDR0获取数据
}

ISR(USART_UDRE_vect) {
    UDR0 = data; // 发送数据到数据寄存器UDR0
    data = 0; // 清除缓冲
    UCSR0B &= ~(1<<UDRIE0); // 立即关闭数据空中断寄存器
}
