#include "protocol.h"
// 声明：本实例由任羽飞编写，未经允许禁止抄袭，转载请说明出处
static char line[LINE_BUFFER_SIZE]; // 定义一个代码行缓冲区

void protocol_main_loop()
{
    uint8_t c;
    uint8_t char_counter = 0;
    for (;;)
    {

        // 从串口读取代码行
        while ((c = serial_read()) != SERIAL_NO_DATA)
        {            
            if ((c == '\r') || (c == '\n'))
            {
                line[char_counter] = 0;
                if (line[0] == '$')
                {
                    // 如果是$开头的命令，执行系统命令
                    // system_execute_line(line);
                }
                else
                {
                    // 如果是G代码，执行Gdaim命令
                    // gc_execute_line();
                }
                char_counter = 0;
            }
            else
            {
                line[char_counter++] = c;
            }
        }
    }
}