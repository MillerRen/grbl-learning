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

#include "grbl.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1) // 定义接收缓冲区环形队列长度
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1) // 定义发送缓冲区队列长度

uint8_t serial_rx_buffer[RX_RING_BUFFER]; // 定义串口接收环形队列
uint8_t serial_rx_buffer_head = 0; // 定义串口接收环形队列头指针
volatile uint8_t serial_rx_buffer_tail = 0; // 定义串口接收环形队列尾指针

uint8_t serial_tx_buffer[TX_RING_BUFFER]; // 定义串口发送环形队列
uint8_t serial_tx_buffer_head = 0; // 定义串口发送环形队列头指针
volatile uint8_t serial_tx_buffer_tail = 0; // 定义串口发送环形队列尾指针


// 返回串口读缓冲区可用字节数。
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // 临时变量暂存尾指针优化volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// 返回串口读缓冲区已用的字节数。
// 注意：已废弃。不再被使用除非在config.h中开启了经典状态报告。
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // 临时变量暂存尾指针优化volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// 返回串口发送缓冲区已用的字节数。
// 注意：没有用到除非为了调试和保证串口发送缓冲区没有瓶颈。
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}

// 串口初始化
void serial_init()
{
  // 设置波特率
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // 关闭波特率倍增器。 - 旨在Uno xxx上需要。
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // 波特率高的波特率倍增器开启，即115200
  #endif
  // 波特率是比较大的数字，需要两个8位寄存器存放
  UBRR0H = UBRR0_value >> 8; // 高8位右移到低8位，放入高8位寄存器，右移不会改变源数值
  UBRR0L = UBRR0_value; // 第八位直接放入低8位寄存器

  // 启用接收，发送和接收完成一个字节的中断
  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

  // 默认协议是8位，无奇偶校验，1个停止位
}


// 写入一个字节到串口发送缓冲区。被主程序调用。
void serial_write(uint8_t data) {
  // 计算下一个头指针，如果已经到达最大值，移到开始，形成环形
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // 等待，直到缓冲区有空间
  while (next_head == serial_tx_buffer_tail) {
    // 代办：重构st_prep_tx_buffer()调用，在长打印期间在这里执行。
    if (sys_rt_exec_state & EXEC_RESET) { return; } // 只检查终止防止死循环。
  }

  // 储存数据并向前移动头指针
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // 开启数据寄存器为空的中断，确保串口发送流运行。
  UCSR0B |=  (1 << UDRIE0); // 因为发送队列为空的时候中断处理程序会关闭数据寄存器为空的中断
}


// 数据寄存器为空的中断处理
ISR(SERIAL_UDRE)
{
  // 由于环形队列尾指针中断和主程序都会使用，有可能导致数据读取时，指针已经发生了变化，
  // 存在不稳定性，所以要用临时变量暂存，增加读取时的稳定性。
  uint8_t tail = serial_tx_buffer_tail; // 临时变量暂存 serial_tx_buffer_tail (为volatile优化)

  // 从缓冲区发送一个字节到串口
  UDR0 = serial_tx_buffer[tail];

  // 更新尾指针位置，如果已经到达顶端，返回初始位置，形成环形
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // 如果环形队列为空，关闭串口数据寄存器为空的中断，阻止继续发送串口流
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}


// 获取串口接收缓冲区的第一个字节。被主程序调用。
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // 临时变量暂存 serial_rx_buffer_tail (优化volatile)
  if (serial_rx_buffer_head == tail) { // 如果接收环形队列为空，则设置结束符号
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail]; // 从接受环形队列取一个字节

    tail++; // 更新尾指针
    if (tail == RX_RING_BUFFER) { tail = 0; } // 环形
    serial_rx_buffer_tail = tail;

    return data;
  }
}

// 串口数据接收中断处理
ISR(SERIAL_RX)
{
  uint8_t data = UDR0; // 从串口数据寄存器取出数据
  uint8_t next_head; // 初始化下一个头指针

  // 直接从串行流中选取实时命令字符。这些字符不被传递到主缓冲区，但是它们设置了实时执行的系统状态标志位。
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // 调用运动控制重置程序
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // 设置为 true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // 设置为 true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // 设置为 true
    default :
      if (data > 0x7F) { // 实时控制都是扩展的ASCII字符
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // 设置为 true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // 阻止所有其他状态，调用运动取消。
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          // 以下为实时覆盖命令
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // 除了上面已知的实时命令，其他的ASCII扩展字符都被丢掉
      } else { // 其他的字符被认为都是G代码，会被写入到主缓冲区
        next_head = serial_rx_buffer_head + 1; // 更新临时头指针
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // 写入到接收缓冲区，直到它满了为止。
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}

// 重置并清空串口读缓冲区数据。用于急停和重置。
void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}
