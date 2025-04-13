/*
  serial.c - 为了通过串口接收或发送字节提供底层函数
  Grbl 的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef serial_h
#define serial_h


#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128 // 定义串口接收缓冲区大小
#endif
#ifndef TX_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define TX_BUFFER_SIZE 112 
  #else
    #define TX_BUFFER_SIZE 104 // 定义串口发送缓冲区大小
  #endif
#endif

#define SERIAL_NO_DATA 0xff // 定义串口数据结束符号


void serial_init(); // 串口初始化

// 写入一个字节到串口发送缓冲区。被主程序调用。
void serial_write(uint8_t data);

// 获取串口接收缓冲区的第一个字节。被主程序调用。
uint8_t serial_read();

// 重置并清空串口读缓冲区数据。用于急停和重置。
void serial_reset_read_buffer();

// 返回串口读缓冲区可用字节数。
uint8_t serial_get_rx_buffer_available();

// 返回串口读缓冲区已用的字节数。
// 注意：已废弃。不再被使用除非在config.h中开启了经典状态报告。
uint8_t serial_get_rx_buffer_count();

// 返回串口发送缓冲区已用的字节数。
// 注意：没有用到除非为了调试和保证串口发送缓冲区没有瓶颈。
uint8_t serial_get_tx_buffer_count();

#endif
