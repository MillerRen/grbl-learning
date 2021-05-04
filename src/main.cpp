// #include "grbl.h"

// UCSRnA and UCSRnB and UCSRnC: USART Control and Status Register n A 串口控制与状态寄存器
// UDRn: USART I/O Data Register n 串口IO数据寄存器
// UBRRnL and UBRRnH – USART Baud Rate Registers 串口波特率寄存器
#include "stdlib.h"
#include "stdint.h"
#include "avr/io.h"
#include "avr/interrupt.h"

#define BAUD_RATE 115200

unsigned char data[] = "hello world \n";

int main (void) {
    uint16_t UBBRn_value = F_CPU/(8*BAUD_RATE)-1; // 波特率计算公式
    UCSR0A |= (1<<U2X0); // 开启二倍传输，误差更小
    UBRR0H = UBBRn_value >> 8; // 右移8位得到高位
    UBRR0L = UBBRn_value; // 低位直接存
    
    UCSR0B |= (1<<TXEN0); // 使能串口发送

    int i = 0;
    for (;;)
    {
        i = 0;
        while (data[i]!=0)
        {
            while(!(UCSR0A&(1<<UDRE0))); // 数据寄存器是否为空才写入
            UDR0 = data[i];
            i++;
        }
        
    }
    return 0;
}

