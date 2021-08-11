// grbl串口操作案例
// 声明：本实例由任羽飞编写，禁止抄袭，转载请说明出处
#include "../lib/protocol/src/protocol.h"

int main (void) {

    serial_init();
    sei(); 
    

    while (1)
    {
        
        protocol_main_loop();

    }
    
}

