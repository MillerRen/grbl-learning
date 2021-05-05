// grbl串口操作案例
// 声明：本实例由任羽飞编写，禁止抄袭，转载请说明出处
#include "../lib/serial/src/serial.h"

#define LINE_BUFFER_SIZE 80

static char line[LINE_BUFFER_SIZE]; // 定义一个代码行缓冲区

int main (void) {
    serial_init();
    sei();
    uint8_t c;
    uint8_t char_counter = 0;   
    

    while (1)
    {
        // 从串口读取代码行
        while((c=serial_read())!=SERIAL_NO_DATA) {
            line[char_counter++] = c;
        }
        if (char_counter) {
            // 发送读取的代码行到串口
            printString(line);
        }
        char_counter = 0;   
        
        
    }
    
}

